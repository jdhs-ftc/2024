package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.hardware.configuration.LynxConstants.EXPANSION_HUB_PRODUCT_NUMBER
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team.BLUE
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign


@TeleOp(name = "Teleop Field Centric")
@Config
//@Photon
class TeleopActions : ActionOpMode() {
    // Declare a PIDF Controller to regulate heading
    private val headingPIDJoystick = PIDFController.PIDCoefficients(0.6, 0.0, 1.0)
    private val joystickHeadingController = PIDFController(headingPIDJoystick)

    val allHubs by lazy { hardwareMap.getAll<LynxModule>(LynxModule::class.java) }
    val controlHub by lazy {
        allHubs.find { // search through all LynxModules (chub, ex hub, shub)
            it.revProductNumber == EXPANSION_HUB_PRODUCT_NUMBER // check that it's an expansion hub not a servo hub
                    && it.isParent // check that it's directly connected over USB
                    && LynxConstants.isEmbeddedSerialNumber(it.serialNumber) // check that it's a control hub's integrated expansion hub
        } as LynxModule // ensure it's non-null
    }
    val expansionHub by lazy {
        allHubs.find { // search through all LynxModules (chub, ex hub, shub)
            it.revProductNumber == EXPANSION_HUB_PRODUCT_NUMBER // check that it's an expansion hub not a servo hub
                    && (!it.isParent // check that it's NOT connected over usb
                    || (it.isParent && !LynxConstants.isEmbeddedSerialNumber(it.serialNumber)))
            // or that it is connected over USB but isn't an integrated hub
        } as LynxModule // ensure it's non-null
    }

    val startPose = if ((System.currentTimeMillis() - PoseStorage.poseUpdatedTime) / 1000 < 40) { // if auto ended less than 40 seconds ago
        PoseStorage.currentPose // use pose from end of auto
    }  else {
        if (PoseStorage.currentTeam == BLUE) {
            Pose2d(0.0, 0.0, Math.toRadians(-90.0)) // otherwise assume started facing forward
        } else {
            Pose2d(0.0, 0.0, Math.toRadians(90.0))
        }
    }

    val drive by lazy { PinpointDrive(hardwareMap, startPose) }
    val motorControl by lazy { MotorControl(hardwareMap) }
    val motorActions by lazy { MotorActions(motorControl) }


    val loopTime = ElapsedTime()
    val currentGamepad1 = Gamepad()
    val currentGamepad2 = Gamepad()
    val previousGamepad1 = Gamepad()
    val previousGamepad2 = Gamepad()

    var speed = 1.0
    var targetHeading = startPose.heading
    var fieldCentric = true
    var drivingEnabled = true

    var showMotorTelemetry = true
    var showStateTelemetry = true
    var showLoopTimes = true
    var showPoseTelemetry = true
    var showCameraTelemetry = false

    val loopTimeAvg = LinkedList<Double>()
    val timeSinceDriverTurned = ElapsedTime()

    val sampleMode = false


    override fun runOpMode() {
        //  Initialization Period


        //EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive // init with by lazy

        // Enable Bulk Caching
        allHubs.forEach { it.bulkCachingMode == BulkCachingMode.MANUAL }


        joystickHeadingController.setInputBounds(-Math.PI, Math.PI)

        // Telemetry Init
        telemetry.msTransmissionInterval = 50
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        waitForStart()

        if (isStopRequested) return
        // Motor Init
        motorControl // init with by lazy
        motorActions


        // Run Period
        while (opModeIsActive() && !isStopRequested) {
            // Reset measured loop time
            loopTime.reset()
            val packet = TelemetryPacket()
            // Reset bulk cache
            allHubs.forEach { it.clearBulkCache() }

            // This lets us do reliable rising-edge detection, even if it changes mid-loop
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)

            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            padReleased = true

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            val padSlowMode = gamepad1.circle
            val padFastMode = gamepad1.right_bumper
            val padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left

            // Misc/Obscure
            val padCameraAutoAim = gamepad1.right_stick_button

            // Extra Settings
            val pad1ExtraSettings = gamepad1.share
            val pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left // 1 rumble blue, 2 rumble red
            val pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up


            // Gamepad 2
            // Presets/Automated
            val padWallPreset = (gamepad2.x && !previousGamepad2.x)
            val padWallPresetRelease = !gamepad2.x
            padReleased = padReleased && padWallPresetRelease
            val padHighPreset = gamepad2.y
            val padMidPreset = gamepad2.b
            val padLowPreset = gamepad2.a

            val padDepositClawToggle =
                (gamepad2.right_bumper && !previousGamepad2.right_bumper) //|| (gamepad1.square && !previousGamepad1.square);
            //val padExtendoClawToggle = (gamepad2.left_bumper && !previousGamepad2.left_bumper)
            //val padArmToggle = (gamepad2.right_trigger > 0.25 && !(previousGamepad2.right_trigger > 0.25))
            val padArmUpFull = (gamepad2.right_bumper && !(previousGamepad2.right_bumper))

            val padExtendoArmDown =
                (gamepad2.right_trigger > 0.25 && !(previousGamepad2.right_trigger > 0.25)) // trigger rising edge
            val padExtendoGrabAndArmUp = !(gamepad2.right_trigger > 0.25) // trigger falling edge
            padReleased = padReleased && padExtendoGrabAndArmUp

            val padDepositChamber = (gamepad2.left_trigger > 0.25 && !(previousGamepad2.left_trigger > 0.25))
            val padDepositChamberRelease = (gamepad2.left_trigger < 0.25)
            padReleased = padReleased && padDepositChamberRelease

            //val padDepositArmDump =

            // carson mixes up lefts from rights;
            // grip tape?
            // use triggers?

            // Manual Control
            var padDepositControl = -gamepad2.left_stick_y.toDouble()
            padDepositControl = sign(padDepositControl) * abs(padDepositControl).pow(2)
            var padExtendoControl = -gamepad2.right_stick_y.toDouble()
            padExtendoControl = sign(padExtendoControl) * abs(padExtendoControl).pow(2)
            val padSlideControlMultiplier = 20.0

            val padTransfer = gamepad2.left_bumper && !previousGamepad2.left_bumper
            val padTransferRelease = !gamepad2.left_bumper

            padReleased = padReleased && padTransferRelease


            // Misc
            val padForceDown = gamepad2.left_stick_button || gamepad2.right_stick_button


            // Update the speed
            speed = if (padSlowMode) {
                .35
            } else if (padFastMode) {
                1.5
            } else {
                1.0 //.8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (PoseStorage.currentTeam != BLUE) { // Team is declared and saved there for auto
                    drive.pose = Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0))
                } else {
                    drive.pose = Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0))
                }
                targetHeading = drive.pose.heading
                gamepad1.rumbleBlips(1) // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2)
                    } else {
                        gamepad1.rumbleBlips(1)
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1)
                        PoseStorage.currentTeam = PoseStorage.Team.BLUE
                    } else {
                        gamepad1.rumbleBlips(2)
                        PoseStorage.currentTeam = PoseStorage.Team.RED
                    }
                }
            }


            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            var input = Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed + -gamepad2.right_stick_x * 0.2
            )

            var rotationAmount = drive.pose.heading.inverse() // inverse it
            if (fieldCentric && !padCameraAutoAim) {
                rotationAmount =
                    if (PoseStorage.currentTeam == BLUE) { // Depending on which side we're on, the forward angle from driver's perspective changes
                        rotationAmount + Math.toRadians(-90.0)
                    } else {
                        rotationAmount + Math.toRadians(90.0)
                    }
                input = rotationAmount * input // rotate the input by the rotationamount
                // (rotationAmount MUST go first here)
            }
            val controllerHeading = Vector2d(-gamepad1.right_stick_y.toDouble(), -gamepad1.right_stick_x.toDouble())



            if (drivingEnabled) {
                var headingInput: Double
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    headingInput = (gamepad1.left_trigger - gamepad1.right_trigger) * speed * 0.50
                    targetHeading = drive.pose.heading
                    timeSinceDriverTurned.reset()
                } else {
                    // Set the target heading for the heading controller to our desired angle
                    if (controllerHeading.norm() > 0.4) { // if the joystick is tilted more than 0.4 from the center,
                        // Cast the angle based on the angleCast of the joystick as a heading
                        targetHeading = if (PoseStorage.currentTeam == BLUE) {
                            controllerHeading.angleCast().plus(Math.toRadians(-90.0))
                        } else {
                            controllerHeading.angleCast().plus(Math.toRadians(90.0))
                        }
                    }

                    joystickHeadingController.targetPosition = targetHeading.toDouble()
                    // Set the desired angular velocity to the heading controller output plus angular
                    // velocity feedforward
                    if (timeSinceDriverTurned.milliseconds() > 250) {
                        headingInput = ((joystickHeadingController.update(drive.pose.heading.log())
                                * MecanumDrive.PARAMS.kV
                                * MecanumDrive.PARAMS.trackWidthTicks))
                    } else {
                        headingInput = 0.0
                        targetHeading = drive.pose.heading
                    }
                }
                drive.setDrivePowers(
                    PoseVelocity2d(
                        Vector2d(
                            input.x,
                            input.y
                        ),
                        headingInput
                    )
                )
            }


            // LIFT CONTROL/FSM


            // Slide (Manual)
            // TODO: abstract this?
            if (motorControl.deposit.targetPosition > 1600 && padDepositControl > 0) {
                motorControl.deposit.targetPosition = 1600.0
            } else if (motorControl.deposit.targetPosition <= 20 && padDepositControl < 0) {
                if (padForceDown) {
                    motorControl.deposit.findZero()
                }
                motorControl.deposit.targetPosition = 20.0
            } else {
                motorControl.deposit.targetPosition += (padDepositControl * padSlideControlMultiplier)
            }

            if (motorControl.extendo.targetPosition > 1430 && padExtendoControl > 0) { // previously 1530
                motorControl.extendo.targetPosition = 1430.0
            } else if (motorControl.extendo.targetPosition <= 5 && padExtendoControl < 0) {
                if (padForceDown) {
                    motorControl.extendo.findZero()
                }
                motorControl.extendo.targetPosition = 5.0
            } else {
                motorControl.extendo.targetPosition += (padExtendoControl * padSlideControlMultiplier)
            }

            if (padDepositClawToggle) {
                motorControl.depositClaw.toggle()
            }
            /*
            if (padExtendoClawToggle) {
                motorControl.extendoClaw.toggle()
            }*

             */

            if (sampleMode) {
                if (padArmUpFull) {
                    run(
                        UniqueAction(
                            SequentialAction(
                                motorActions.extendo.moveDown(),
                                motorActions.deposit.moveDown(),
                                motorActions.depositArm.moveDown(),
                                motorActions.extendoArm.moveFullUp(),
                                // wait for extendo arm to get to target
                                // maybe use gamepad here?
                                SleepAction(0.5),
                                motorActions.extendoClaw.open(),
                                SleepAction(0.1), // wait for the claw to open
                                motorActions.extendoArm.moveUp(),
                                SleepAction(0.1) // wait for the extendo arm to finish
                                        // moving to just above the ground position


                            )
                        )
                    )
                }
            }


            if (padHighPreset) {
                motorControl.deposit.targetPosition = 1600.0
            }
            if (padLowPreset) {
                motorControl.deposit.targetPosition = 20.0
                motorControl.extendo.targetPosition = 20.0
            }

            if (padExtendoArmDown) {
                println("12087 extendo arm down")
                println("12087 extendo arm dumping ${motorControl.extendoArm.fullyUp}")
                println("12087 extendo arm position ${motorControl.extendoArm.position}")
                if (motorControl.extendoArm.fullyUp) {
                    run(
                        UniqueAction(
                            motorActions.extendoArm.moveUp()
                        )
                    )
                } else {
                    run(
                        UniqueAction(
                            SequentialAction(
                                motorActions.extendoClaw.open(), // open claw
                                SleepAction(0.05),
                                motorActions.extendoArm.moveDown(), // move to ground
                                waitForPadRelease(), // wait until trigger releases
                                motorActions.extendoClaw.close(), // close claw
                                SleepAction(0.5), // TODO tune
                                motorActions.extendoArm.moveUp() // move claw to "clears ground bar" pos
                            )
                        )
                    )
                }
            }
            if (padWallPreset) {
                run(
                    UniqueAction(
                        SequentialAction(
                            motorActions.depositMoveWall(),
                            RaceParallelAction(
                                waitForPadRelease(),
                            ),
                            motorActions.depositPickupWall()
                        )
                    )
                )
            }

            if (padDepositChamber) {
                run(
                    UniqueAction(
                        SequentialAction(
                            motorActions.depositMoveChamber(),
                            waitForPadRelease(),
                            motorActions.depositScoreChamber()
                        )
                    )
                )
            }

            if (padTransfer) {
                run(
                    UniqueAction(
                        SequentialAction(
                            motorActions.moveTransfer(),
                            waitForPadRelease(),
                            motorActions.grabTransferReturn()
                        )
                    )
                )
            }


            /*
            gamepad2.rumble(
                if (motorControl.dColor.color != Color.NONE && !motorControl.depositClaw.closed) {
                    0.4
                } else {
                    0.0
                },
                0.0,
                Gamepad.RUMBLE_DURATION_CONTINUOUS
            )

             */


            // update RR, update motor controllers


            // TELEMETRY
            Drawing.drawRobot(
                packet.fieldOverlay(),
                drive.pose
            )
            // why is this commented out
            // what was I even trying to do with this
            // did I think pose was in ticks??
            // this would technically have worked for that ig?????
            // or did roadrunner use this?
            // - j5155, 2024,
            // with no memory of what this was (which means it was probably written in 2022)
            // new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

            updateAsync(packet)
            packet.put("448", loopTime.milliseconds())
            drive.updatePoseEstimate()
            packet.put("450", loopTime.milliseconds())
            motorControl.update()
            packet.put("452", loopTime.milliseconds())
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            val loopTimeMs = loopTime.milliseconds()
            loopTimeAvg.add(loopTimeMs)
            while (loopTimeAvg.size > 1000) {
                loopTimeAvg.removeFirst()
            }

            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---")
                telemetry.addData("x", drive.pose.position.x)
                telemetry.addData("y", drive.pose.position.y)
                telemetry.addData("heading", drive.pose.heading.log())
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---")
                telemetry.addData("loopTimeMs", loopTimeMs)
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs)
                telemetry.addData(
                    "LoopAverage ",
                    loopTimeAvg.sum() / loopTimeAvg.size
                )
            }
            if (showMotorTelemetry) {
                telemetry.addLine("--- Motors ---")
                telemetry.addData("extendoTarget", motorControl.extendo.targetPosition)
                telemetry.addData("extendoPosition", motorControl.extendo.position)
                telemetry.addData("depositTarget", motorControl.deposit.targetPosition)
                telemetry.addData("depositPosition", motorControl.deposit.position)
                telemetry.addData("extendoOffset", motorControl.extendo.encoderOffset)
                telemetry.addData("extendoResetting", motorControl.extendo.resetting)
                telemetry.addData("depositClawPosition", motorControl.depositClaw.position)
                telemetry.addData("dColor", motorControl.dColor.color)
                telemetry.addData("dumping",motorControl.extendoArm.fullyUp)
                telemetry.addData("depositArmPosition",motorControl.depositArm.position)


                //telemetry.addData("extendoClawPos", motorControl.extendoClaw.getPosition());
                //telemetry.addData("depositClawPos", motorControl.depositClaw.getPosition());
            }


            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---")
                telemetry.addData("actions", runningActions)
            }
            telemetry.update()
        }
    }

    fun waitForPadRelease(): Action {
        return Action {
            return@Action !padReleased
        }
    }

    companion object {
        var padReleased = true
    }



}

