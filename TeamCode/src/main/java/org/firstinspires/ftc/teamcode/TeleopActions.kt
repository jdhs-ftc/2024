package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.TeleopActions.Input
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.util.LinkedList
import java.util.function.Consumer
import kotlin.math.log10
import kotlin.math.pow
import kotlin.math.sqrt


@TeleOp(name = "Teleop Field Centric")
@Config
class TeleopActions : ActionOpMode() {
    // Declare a PIDF Controller to regulate heading
    private val headingPIDJoystick = PIDFController.PIDCoefficients(0.6, 0.0, 1.0)
    private val joystickHeadingController = PIDFController(headingPIDJoystick)
    var speed: Double = 0.0
    var targetHeading: Rotation2d = PoseStorage.currentPose.heading
    var controlHub: LynxModule? = null
    var expansionHub: LynxModule? = null
    var drive: PinpointDrive? = null
    var motorControl: MotorControl? = null
    var motorActions: MotorActions? = null

    var fieldCentric = true

    val loopTime = ElapsedTime()
    val currentGamepad1 = Gamepad()
    val currentGamepad2 = Gamepad()
    val previousGamepad1 = Gamepad()
    val previousGamepad2 = Gamepad()

    var showMotorTelemetry = true
    var showStateTelemetry = true
    var showLoopTimes = true
    var showPoseTelemetry = true
    var showCameraTelemetry = false


    var drivingEnabled = true
    var actionRunning = false
    var suspendSet = false
    var loopTimeAvg = LinkedList<Double>()

    var loopTimeBeforeUpdate = 0.0
    var loopTimeAfterUpdate = 0.0

    var timeSinceDriverTurned = ElapsedTime()


    override fun runOpMode() {
        //  Initialization Period

        // Enable Bulk Caching

        val allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        for (module in allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }
        controlHub = allHubs.get(0)

        //EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive = PinpointDrive(hardwareMap, PoseStorage.currentPose)


        joystickHeadingController.setInputBounds(-Math.PI, Math.PI)

        // Telemetry Init
        telemetry.msTransmissionInterval = 50
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        waitForStart()

        if (isStopRequested()) return
        // Motor Init
        motorControl = MotorControl(hardwareMap)
        motorActions = MotorActions(motorControl!!)


        // Run Period
        while (opModeIsActive() && !isStopRequested()) {
            // Reset measured loop time
            loopTime.reset()
            // Reset bulk cache
            allHubs.forEach(Consumer { obj: LynxModule? -> obj!!.clearBulkCache() })

            // This lets us do reliable rising edge detection, even if it changes mid loop
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)

            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            val padSlowMode = gamepad1.left_bumper
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
            val padWallPresetRelease = (!gamepad2.x && previousGamepad2.x)
            val padHighPreset = gamepad2.y
            val padMidPreset = gamepad2.b
            val padLowPreset = gamepad2.a

            val padDepositClawToggle =
                (gamepad2.right_bumper && !previousGamepad2.right_bumper) //|| (gamepad1.square && !previousGamepad1.square);
            //val padExtendoClawToggle = (gamepad2.left_bumper && !previousGamepad2.left_bumper)
            val padArmToggle = (gamepad2.right_trigger > 0.25 && !(previousGamepad2.right_trigger > 0.25))
            val padArmUpFull = (gamepad2.right_bumper && !(previousGamepad2.right_bumper))

            val padExtendoArmDown = (gamepad2.right_trigger > 0.25 && !(previousGamepad2.right_trigger > 0.25)) // trigger rising edge
            val padExtendoGrabAndArmUp = (!(gamepad2.right_trigger > 0.25) && (previousGamepad2.right_trigger > 0.25)) // trigger falling edge

            // carson mixes up lefts from rights;
            // grip tape?
            // use triggers?

            // Manual Control
            val padDepositControl = -gamepad2.left_stick_y.toDouble()
            val padExtendoControl = -gamepad2.right_stick_y.toDouble()
            val padSlideControlMultiplier = 10.0


            // Misc
            val padForceDown = gamepad2.dpad_down && gamepad2.options

            val padResetExtendo = gamepad2.dpad_up && gamepad2.options


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
                if (PoseStorage.currentTeam != PoseStorage.Team.BLUE) { // Team is declared and saved there for auto
                    drive!!.pose = Pose2d(drive!!.pose.position.x, drive!!.pose.position.y, Math.toRadians(90.0))
                } else {
                    drive!!.pose = Pose2d(drive!!.pose.position.x, drive!!.pose.position.y, Math.toRadians(-90.0))
                }
                targetHeading = drive!!.pose.heading
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
                -gamepad1.left_stick_x * speed
            )

            //Pose2d poseEstimate = drive.pose;
            var rotationAmount = -drive!!.pose.heading.log() // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric && !padCameraAutoAim) {
                if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) { // Depending on which side we're on, the color changes
                    rotationAmount = rotationAmount - Math.toRadians(90.0)
                } else {
                    rotationAmount = rotationAmount + Math.toRadians(90.0)
                }
                input = Rotation2d.fromDouble(rotationAmount).times(
                    Vector2d(
                        input.x,
                        input.y
                    )
                ) // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            val controllerHeading = Vector2d(-gamepad1.right_stick_y.toDouble(), -gamepad1.right_stick_x.toDouble())

            if (drivingEnabled) {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    drive!!.setDrivePowers(
                        PoseVelocity2d(
                            Vector2d(
                                input.x,
                                input.y
                            ),
                            (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                        )
                    )
                    targetHeading = drive!!.pose.heading
                    timeSinceDriverTurned.reset()
                } else {
                    // Set the target heading for the heading controller to our desired angle
                    if (sqrt(controllerHeading.x.pow(2.0) + controllerHeading.y.pow(2.0)) > 0.4) {
                        // Cast the angle based on the angleCast of the joystick as a heading
                        if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(-90.0))
                        } else {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(90.0))
                        }
                    }

                    joystickHeadingController.targetPosition = targetHeading.toDouble()

                    var headingInput: Double
                    // Set the desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    if (timeSinceDriverTurned.milliseconds() > 250) {
                        headingInput = ((joystickHeadingController.update(drive!!.pose.heading.log())
                                * MecanumDrive.PARAMS.kV
                                * MecanumDrive.PARAMS.trackWidthTicks))
                        drive!!.setDrivePowers(
                            PoseVelocity2d(
                                Vector2d(
                                    input.x,
                                    input.y
                                ),
                                headingInput
                            )
                        )
                    } else {
                        headingInput = 0.0
                        targetHeading = drive!!.pose.heading
                        drive!!.setDrivePowers(
                            PoseVelocity2d(
                                Vector2d(
                                    input.x,
                                    input.y
                                ),
                                headingInput
                            )
                        )
                    }
                }
            }


            // LIFT CONTROL/FSM


            // Slide (Manual)
            // TODO: abstract this?
            if (motorControl!!.deposit.targetPosition > 1600 && padDepositControl > 0) {
                motorControl!!.deposit.targetPosition = 1600.0
            } else if (motorControl!!.deposit.targetPosition <= 20 && padDepositControl < 0 && !padForceDown) {
                motorControl!!.deposit.findZero()
                motorControl!!.deposit.targetPosition = 20.0
            } else {
                motorControl!!.deposit.targetPosition += (padDepositControl * padSlideControlMultiplier)
            }

            if (motorControl!!.extendo.targetPosition > 1530 && padExtendoControl > 0) {
                motorControl!!.extendo.targetPosition = 1530.0
            } else if (motorControl!!.extendo.targetPosition <= 20 && padExtendoControl < 0 && !padForceDown) {
                motorControl!!.extendo.findZero()
                motorControl!!.extendo.targetPosition = 20.0
            } else {
                motorControl!!.extendo.targetPosition += (padExtendoControl * padSlideControlMultiplier)
            }

            if (padDepositClawToggle) {
                motorControl!!.depositClaw.toggle();
            }
            /*
            if (padExtendoClawToggle) {
                motorControl!!.extendoClaw.toggle()
            }*

             */

            if (padArmToggle) {
                motorControl!!.extendoArm.toggle()
            }
            if (padArmUpFull) {
                run(UniqueAction(SequentialAction(


                )))
                motorControl!!.extendoArm.position = 0.6
            }


            if (padHighPreset) {
                motorControl!!.deposit.targetPosition = 1600.0
            }
            if (padMidPreset) {
                motorControl!!.extendo.targetPosition = 1530.0
            }
            if (padLowPreset) {
                motorControl!!.deposit.targetPosition = 20.0
                motorControl!!.extendo.targetPosition = 20.0
            }

            if (padExtendoArmDown) {
                run(UniqueAction(SequentialAction(
                    motorActions!!.extendoClaw.open(), // open claw
                    motorActions!!.extendoArm.moveDown(), // move to ground
                    Action { return@Action !padExtendoGrabAndArmUp }, // wait until trigger releases
                    motorActions!!.extendoClaw.close(), // close claw
                    SleepAction(0.3), // TODO tune
                    motorActions!!.extendoArm.moveUp() // move claw to "clears ground bar" pos
                )))
            }
            if (padWallPreset) {
                run(UniqueAction(SequentialAction(
                    motorActions!!.deposit.setTargetPosition(100.0), // TODO TUNE!
                    motorActions!!.extendoArm.moveUp(),
                    motorActions!!.depositArm.moveUp(),
                    motorActions!!.depositClaw.open(),
                    Action { return@Action !padWallPresetRelease },
                    motorActions!!.extendoClaw.open(),
                    motorActions!!.depositClaw.close(),
                )))
            }






            val colorAlpha = 0.0

            // rumble the gunner controller based on the claw color sensor
            var pad2rumble: Double = if (colorAlpha > 200) { // && !motorControl.extendoClaw.closed) {
                log10(colorAlpha) / 6
            } else {
                0.0
            }
            gamepad2.rumble(pad2rumble, pad2rumble, Gamepad.RUMBLE_DURATION_CONTINUOUS)


            // update RR, update motor controllers


            // TELEMETRY
            val packet = TelemetryPacket()
            Drawing.drawRobot(
                packet.fieldOverlay(),
                drive!!.pose
            ) //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

            updateAsync(packet)
            drive!!.updatePoseEstimate()
            motorControl!!.update()
            FtcDashboard.getInstance().sendTelemetryPacket(packet)

            val loopTimeMs = loopTime.milliseconds()
            loopTimeAvg.add(loopTimeMs)
            while (loopTimeAvg.size > 1000) {
                loopTimeAvg.removeFirst()
            }

            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---")
                telemetry.addData("x", drive!!.pose.position.x)
                telemetry.addData("y", drive!!.pose.position.y)
                telemetry.addData("heading", drive!!.pose.heading.log())
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---")
                telemetry.addData("loopTimeMs", loopTimeMs)
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs)
                telemetry.addData(
                    "LoopAverage ",
                    loopTimeAvg.sum() / loopTimeAvg.size
                )
                telemetry.addData("msBeforeUpdate", loopTimeBeforeUpdate)
                telemetry.addData("msAfterUpdate", loopTimeAfterUpdate)
            }

            if (showMotorTelemetry) {
                telemetry.addLine("--- Motors ---")
                telemetry.addData("extendoTarget", motorControl!!.extendo.targetPosition)
                telemetry.addData("extendoPosition", motorControl!!.extendo.position)
                telemetry.addData("depositTarget", motorControl!!.deposit.targetPosition)
                telemetry.addData("depositPosition", motorControl!!.deposit.position)
                //telemetry.addData("extendoClawPos", motorControl.extendoClaw.getPosition());
                //telemetry.addData("depositClawPos", motorControl.depositClaw.getPosition());
            }


            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---")
                telemetry.addData("actions", runningActions)
            }
            loopTimeBeforeUpdate = loopTime.milliseconds()
            telemetry.update()
            loopTimeAfterUpdate = loopTime.milliseconds()
        }
    }


    // TODO: probably not needed, just make a normal action
    interface Input {
        fun isPressed(): Boolean
    }

    fun waitForInput(input: Input): Action {
        return Action { telemetryPacket: TelemetryPacket -> input.isPressed() }
    }
}

