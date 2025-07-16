package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.NullAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.hardware.configuration.LynxConstants.EXPANSION_HUB_PRODUCT_NUMBER
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.helpers.ASExtraLogging
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.InterruptibleAction
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team.BLUE
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team.RED
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.helpers.UniqueActionQueue
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.round
import kotlin.math.sign


@TeleOp(name = "00 Teleop Field Centric")
@Config
//@Photon
class TeleopActions : ActionOpMode() {

    // Declare a PIDF Controller to regulate heading
    private val headingPIDJoystick = PIDFController.PIDCoefficients(0.6, 0.0, 1.0)
    private val joystickHeadingController = PIDFController(headingPIDJoystick)

    val allHubs: List<LynxModule> by lazy { hardwareMap.getAll<LynxModule>(LynxModule::class.java) }
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

    val startPose: Pose2d = PoseStorage.currentPose
        /*
        if ((System.currentTimeMillis() - PoseStorage.poseUpdatedTime) / 1000 < 400000) {  // DISABLED // if auto ended less than 40 seconds ago
        PoseStorage.currentPose // use pose from end of auto
    }  else {
        if (PoseStorage.currentTeam == BLUE) {
            Pose2d(0.0, 0.0, Math.toRadians(-90.0)) // otherwise assume started facing forward
        } else {
            Pose2d(0.0, 0.0, Math.toRadians(90.0))
        }
    }

         */

    val drive by lazy { OctoQuadDrive(hardwareMap, startPose) } // TODO consider false
    val motorControl by lazy { MotorControl(hardwareMap, lateinit=true) }
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

    val specimenDeposit by lazy { SpecimenDeposit(motorControl, motorActions,6,10) } // prev 5,10

    lateinit var specimenDepositTraj: Action

    var specimenDepositTrajUsed = false

    var hanging = false

    var autoIntakeAction: InterruptibleAction = InterruptibleAction(NullAction())


    override fun runOpMode() {
        ASExtraLogging.start(this)
        //  Initialization Period


        //EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive // init with by lazy
        drive.pose = startPose
        drive.updatePoseEstimate()

        // Enable Bulk Caching
        allHubs.forEach { it.bulkCachingMode == BulkCachingMode.MANUAL }


        joystickHeadingController.setInputBounds(-Math.PI, Math.PI)

        // Telemetry Init
        telemetry.msTransmissionInterval = 50
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry, LogTelemetry())


        motorControl // init with by lazy, NOTE THIS DOESN'T ACTUALLY INIT IT, BECAUSE LATEINIT
        motorActions

        specimenDeposit // init with by lazy
        specimenDepositTraj = specimenDeposit.genTrajectory(drive)


        waitForStart()

        if (isStopRequested) return

        // Motor Init
        motorControl.init()
        run(motorActions.depositArm.moveDown()) // SHOULD HOPEFULLY WORK




        // Run Period
        while (opModeIsActive() && !isStopRequested) {
            ASExtraLogging.loop(this)
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
            val padSlowMode = false // NOT MAPPED //gamepad1.right_bumper
            val padFastMode = false // NOT MAPPED
            val padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left

            val padFaceDown = gamepad1.a
            val padFaceRight = gamepad1.b
            val padFaceLeft = gamepad1.x
            val padFaceUp = gamepad1.y


            // Misc/Obscure
            val padCameraAutoAim = gamepad1.right_stick_button

            // Extra Settings
            val pad1ExtraSettings = gamepad1.share
            val pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left // 1 rumble blue, 2 rumble red
            val pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up


            // Gamepad 2
            val padDepArm0 = gamepad2.dpad_left && !previousGamepad2.dpad_left
            val padDepArm1 = gamepad2.dpad_right && !previousGamepad2.dpad_right

            val padResetAll = gamepad2.touchpad && !previousGamepad2.touchpad

            val padHang = gamepad2.left_bumper && !previousGamepad2.left_bumper
            val padHangRelease = !gamepad2.left_bumper && previousGamepad2.left_bumper



            // Intake

            val padAutoIntakeStart = currentGamepad2.right_trigger > 0.3 && !(previousGamepad2.right_trigger > 0.3)

            val padIntakeReject = currentGamepad2.right_bumper && !previousGamepad2.right_bumper
            val padIntakeStop = !currentGamepad2.right_bumper && previousGamepad2.right_bumper

            val padIntakeExtendoIn = currentGamepad2.square && !previousGamepad2.square
            val padIntakeExtendoOut = !currentGamepad2.square && previousGamepad2.square


            // Deposit

            val padTopChamber = gamepad2.dpad_up && !previousGamepad2.dpad_up
            val padBottomChamber = gamepad2.dpad_right && !previousGamepad2.dpad_right

            val padMoveChamber = gamepad2.left_trigger > 0.3 && !(previousGamepad2.left_trigger > 0.3)
            val padScoreChamber = !(gamepad2.left_trigger > 0.3) && previousGamepad2.left_trigger > 0.3


            // Auto Tele
            val padAutoDrive = false//gamepad2.triangle && !previousGamepad2.triangle
            val padAutoDriveRelease = false //!gamepad2.triangle
            padReleased = padReleased && padAutoDriveRelease


            val padAutoRightTop = gamepad2.dpad_up && !previousGamepad2.dpad_up
            val padAutoRightBottom = gamepad2.dpad_right && !previousGamepad2.dpad_right
            val padAutoLeftTop = gamepad2.dpad_left && !previousGamepad2.dpad_left
            val padAutoLeftBottom = gamepad2.dpad_down && !previousGamepad2.dpad_down




            // carson mixes up lefts from rights;
            // grip tape?
            // use triggers?

            var padExtendoAndStrafeVector = Vector2d (
                -gamepad2.right_stick_x.toDouble(),
                -gamepad2.right_stick_y.toDouble()
            )

            // Manual Control
            var padDepositControl = -gamepad2.left_stick_y.toDouble()
            padDepositControl = sign(padDepositControl) * abs(padDepositControl).pow(2)
            // EXTENDO CONTROL HAPPENS AFTER DRIVING

            val padSlideControlMultiplier = 20.0

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
                //if (PoseStorage.currentTeam != BLUE) { // Team is declared and saved there for auto
                    drive.pose = Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0))
                /*} else {
                    drive.pose = Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0))
                }

                 */
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
                    if (PoseStorage.currentTeam == RED) {
                        gamepad1.rumbleBlips(1)
                        PoseStorage.currentTeam = BLUE
                        motorControl.topLight.color = Color.BLUE
                    } else {
                        gamepad1.rumbleBlips(2)
                        PoseStorage.currentTeam = RED
                        motorControl.topLight.color = Color.RED
                    }
                }
            }




            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            var input = Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed
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
                padExtendoAndStrafeVector = (Rotation2d.fromDouble(round(rotationAmount.toDouble() / Math.toRadians(90.0)) * Math.toRadians(90.0)).inverse()) * padExtendoAndStrafeVector // TODO: round???
            }
            val controllerHeading = Vector2d(-gamepad1.right_stick_y.toDouble(), -gamepad1.right_stick_x.toDouble())

            var padExtendoControl = padExtendoAndStrafeVector.y
            padExtendoControl = sign(padExtendoControl) * abs(padExtendoControl).pow(2)

            if (motorControl.extendo.targetPosition >= 2000 && padExtendoControl > 0) { // previously 1530
                motorControl.extendo.targetPosition = 2000.0
                input += Vector2d(padExtendoControl * 0.2,0.0)
            } else if (motorControl.extendo.targetPosition <= 5 && padExtendoControl < 0) {
                if (padForceDown) {
                    motorControl.extendo.findZero()
                }
                motorControl.extendo.targetPosition = 5.0
                input += Vector2d(padExtendoControl * 0.2, 0.0)
            } else {
                motorControl.extendo.targetPosition += (padExtendoControl * padSlideControlMultiplier)
            }

            input += Vector2d(0.0, padExtendoAndStrafeVector.x * 0.5)

            var headingInput = 0.0
            if (drivingEnabled) {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1 || gamepad1.left_bumper || gamepad1.right_bumper) {
                    headingInput = (gamepad1.left_trigger - gamepad1.right_trigger) * speed * 0.50

                    if (gamepad1.left_bumper) {
                        headingInput += 0.1
                    }
                    if (gamepad1.right_bumper) {
                        headingInput -= 0.1
                    }
                    targetHeading = drive.pose.heading
                    timeSinceDriverTurned.reset()
                } else {
                    val baseHeading = if (PoseStorage.currentTeam == RED) {
                        Math.toRadians(90.0)
                    } else {
                        Math.toRadians(-90.0)
                    }
                    // Set the target heading for the heading controller to our desired angle
                    if (controllerHeading.norm() > 0.4) { // if the joystick is tilted more than 0.4 from the center,
                        // Cast the angle based on the angleCast of the joystick as a heading
                        targetHeading = controllerHeading.angleCast() + baseHeading
                    }

                    if (padFaceDown) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(180.0)) + baseHeading
                    } else if (padFaceRight) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(-90.0)) + baseHeading
                    } else if (padFaceLeft) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(90.0)) + baseHeading
                    } else if (padFaceUp) {
                        targetHeading = Rotation2d.fromDouble(Math.toRadians(0.0)) + baseHeading
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


                // LIFT CONTROL/FSM


                // Slide (Manual)
                // TODO: abstract this?
                if (motorControl.deposit.targetPosition >= 2950 && padDepositControl > 0) {
                    motorControl.deposit.targetPosition = 2950.0
                } else if (motorControl.deposit.targetPosition <= 20 && padDepositControl < 0) {
                    if (padForceDown) {
                        motorControl.deposit.findZero()
                    }
                    motorControl.deposit.targetPosition = 20.0
                } else {
                    motorControl.deposit.targetPosition += (padDepositControl * padSlideControlMultiplier)
                }



                /*
            if (padExtendoClawToggle) {
                motorControl.extendoClaw.toggle()
            }*

             */


                if (padAutoDrive) {
                    UniqueActionQueue.runningUniqueActions.clear()
                    runningActions.clear()
                    if (specimenDepositTrajUsed) {
                        specimenDeposit.genTrajectory(drive) // sketchy!!
                        specimenDepositTrajUsed = false
                    }
                    run(
                        UniqueAction(
                            SequentialAction(
                                InstantAction {
                                    UniqueActionQueue.shouldQueueUniqueActions = false
                                },
                                motorActions.depositMoveWallTeleop(),
                                RaceParallelAction(
                                    waitForPadRelease(),
                                ),
                                InstantAction {
                                    drivingEnabled = false
                                    specimenDepositTrajUsed = true
                                    drive.pose = Pose2d(Vector2d(34.0, -63.5),Math.toRadians(90.0));
                                },
                                RaceParallelAction(
                                    specimenDepositTraj,
                                    Action {
                                        return@Action !((currentGamepad2.triangle && !previousGamepad2.triangle) || (currentGamepad1.share && !previousGamepad1.share)) // cancel
                                    }
                                ),
                                InstantAction {
                                    drivingEnabled = true
                                    UniqueActionQueue.shouldQueueUniqueActions = true
                                    targetHeading = drive.pose.heading


                                }
                            )
                        )
                    )


                }
            }


            if (padDepArm0) run(motorActions.depositArm.moveDown())
            if (padDepArm1) run(motorActions.depositArm.moveUp())

            if (padHang && !hanging) {
                motorControl.deposit.targetPosition = 2950.0
                run(SequentialAction(
                    { motorControl.deposit.position < 2000.0 },
                    motorActions.depositArm.moveHang()
                ))
            }

            if (padHangRelease && !hanging) {
                motorControl.deposit.targetPosition = 996.0
                drivingEnabled = false
                hanging = true
                motorControl.topLight.color = Color.VIOLET
            }

            if (padHang && hanging) {
                motorControl.deposit.targetPosition = 2950.0
            }

            if (padHangRelease && hanging) {
                hanging = false
                drivingEnabled = true
                motorControl.topLight.color = motorControl.topLight.lastColor
            }

            if (padMoveChamber) run(motorActions.depositMoveChamber())
            if (padScoreChamber) run(motorActions.depositScoreChamberTeleop())


            if (padResetAll) {
                run(
                    ParallelAction(
                        motorActions.extendoArm.moveUp(),
                        motorActions.extendo.moveDown(),
                        motorActions.deposit.moveDown(),
                        motorActions.depositArm.moveDown(),
                        motorActions.depositClawRelease()
                    )
                )
                motorControl.depositClaw.open()
                motorControl.intake.stop()
            }

            if (gamepad2.dpad_down && !previousGamepad2.dpad_down) motorControl.depositClaw.toggle()


            if (padAutoIntakeStart) {
                autoIntakeAction.interrupt()
                updateAsync(packet)
                autoIntakeAction = motorActions.intakeUntilColor(gamepad1, gamepad2)
                run(autoIntakeAction)
            }
            if (padIntakeReject) {
                autoIntakeAction.interrupt()
                updateAsync(packet)
                motorControl.intake.eject()
            }
            if (padIntakeStop) {
                motorControl.intake.stop()
            }

            if (padIntakeExtendoIn) {
                run(UniqueAction(motorActions.intakeIn()))
            }
            if (padIntakeExtendoOut) {
                run(UniqueAction(motorActions.intakeOut()))
            }





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
            if (drivingEnabled) { // if driving's disabled, the trajectory is already updating it
                drive.updatePoseEstimate()
            }
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
                telemetry.addData("targetHeading",targetHeading.toDouble())
                telemetry.addData("headingDeg", Math.toDegrees(drive.pose.heading.log()))
                telemetry.addData("targetHeading", Math.toDegrees(targetHeading.toDouble()))
                telemetry.addData("poseStorageHeading", Math.toDegrees(PoseStorage.currentPose.heading.toDouble()))
                telemetry.addData("headingInput",headingInput)
                telemetry.addData("timeSinceDriverTurned",timeSinceDriverTurned.milliseconds())
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
                telemetry.addData("extendoPower",motorControl.extendo.motor.power)
                telemetry.addData("depositTarget", motorControl.deposit.targetPosition)
                telemetry.addData("depositPosition", motorControl.deposit.position)
                telemetry.addData("extendoResetting", motorControl.extendo.resetting)
                telemetry.addData("depositClawPosition", motorControl.depositClaw.position)
                telemetry.addData("dColor", motorControl.dColor.color)
                telemetry.addData("eColor", motorControl.eColor.color)
                telemetry.addData("dumping",motorControl.extendoArm.mid)
                telemetry.addData("depositArmTargetPosition",motorControl.depositArm.position)
                telemetry.addData("depositArmEncoderPos", motorControl.depositArmEncoder.position)
                telemetry.addData("depositArmEncoderPosDeg", motorControl.depositArmEncoder.posDegrees)
                telemetry.addData(
                    "extendoArmEncoderPosDeg",
                    motorControl.extendoArmEncoder.posDegrees
                )



                //telemetry.addData("extendoClawPos", motorControl.extendoClaw.getPosition());
                //telemetry.addData("depositClawPos", motorControl.depositClaw.getPosition());
            }


            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---")
                telemetry.addData("actions", UniqueActionQueue.runningUniqueActions)
            }
            telemetry.addLine("--- Specimen Deposit Automation --")
            telemetry.addData("cyclesScored (generated)", specimenDeposit.cyclesScored)
            telemetry.addData("cyclesToScore", specimenDeposit.cyclesToScore)
            telemetry.addData("cyclesRuntimeScored (actual)", specimenDeposit.cyclesRuntimeScored)
            telemetry.addData("drivingEnabled",drivingEnabled)
            telemetry.addData("hanging",hanging)
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

