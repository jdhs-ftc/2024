package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController;
import org.firstinspires.ftc.teamcode.motor.MotorActions;
import org.firstinspires.ftc.teamcode.motor.MotorControl;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

@TeleOp(name = "Teleop Field Centric")
@Config
public class TeleopActions extends ActionOpMode {


    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID_JOYSTICK = new PIDFController.PIDCoefficients(0.6, 0.0, 1);
    private final PIDFController joystickHeadingController = new PIDFController(HEADING_PID_JOYSTICK);
    double speed;
    Rotation2d targetHeading = PoseStorage.currentPose.heading;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    public PinpointDrive drive;

    List<Action> runningActions = new ArrayList<>();
    final ElapsedTime loopTime = new ElapsedTime();
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();
    boolean showMotorTelemetry = true;
    boolean showStateTelemetry = true;
    boolean showLoopTimes = true;
    boolean showPoseTelemetry = true;
    boolean showCameraTelemetry = false;

    MotorControl motorControl;
    MotorActions motorActions;
    boolean drivingEnabled = true;
    boolean actionRunning = false;
    boolean suspendSet = false;
    LinkedList<Double> loopTimeAvg = new LinkedList<>();

    double loopTimeBeforeUpdate = 0;
    double loopTimeAfterUpdate = 0;

    ElapsedTime timeSinceDriverTurned = new ElapsedTime();


    @Override
    public void runOpMode() {

        //  Initialization Period

        // Enable Bulk Caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        //EXPANSION_HUB = allHubs.get(1);

        // RoadRunner Init
        drive = new PinpointDrive(hardwareMap, PoseStorage.currentPose);


        joystickHeadingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        waitForStart();

        if (isStopRequested()) return;
        // Motor Init
        motorControl = new MotorControl(hardwareMap);
        motorActions = new MotorActions(motorControl);


        // Run Period

        while (opModeIsActive() && !isStopRequested()) {
            // Reset measured loop time
            loopTime.reset();
            // Reset bulk cache
            allHubs.forEach(LynxModule::clearBulkCache);

            // This lets us do reliable rising edge detection, even if it changes mid loop
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // CONTROLS

            // Gamepad 1
            // Driving Modifiers
            boolean padSlowMode = gamepad1.left_bumper;
            boolean padFastMode = gamepad1.right_bumper;
            boolean padResetPose = gamepad1.dpad_left && !previousGamepad1.dpad_left;

            // Misc/Obscure
            boolean padCameraAutoAim = gamepad1.right_stick_button;

            // Extra Settings
            boolean pad1ExtraSettings = gamepad1.share;
            boolean pad1ExTeamSwitch = gamepad1.dpad_left && !previousGamepad1.dpad_left; // 1 rumble blue, 2 rumble red
            boolean pad1ExToggleFieldCentric = gamepad1.dpad_up && !previousGamepad1.dpad_up;


            // Gamepad 2
            // Presets/Automated
            boolean padHalfCycle = gamepad2.left_trigger > 0.25;
            boolean padFullCycle = gamepad2.right_trigger > 0.25 || gamepad1.circle;

            boolean padHighPreset = gamepad2.y;
            boolean padMidPreset = gamepad2.b;
            boolean padLowPreset = gamepad2.a;

            boolean padDepositClawToggle = (gamepad2.right_bumper && !previousGamepad2.right_bumper); //|| (gamepad1.square && !previousGamepad1.square);
            boolean padExtendoClawToggle = (gamepad2.left_bumper && !previousGamepad2.left_bumper);
            boolean padArmToggle = (gamepad2.right_trigger > 0.25 && !(previousGamepad2.right_trigger > 0.25));
            boolean padArmUpFull = (gamepad2.left_trigger > 0.25 && !(previousGamepad2.left_trigger > 0.25));
            // carson mixes up lefts from rights;
            // grip tape?
            // use triggers?

            // Manual Control
            double padDepositControl = -gamepad2.left_stick_y;
            double padExtendoControl = -gamepad2.right_stick_y;
            double padSlideControlMultiplier = 10;


            // Misc
            boolean padForceDown = gamepad2.dpad_down && gamepad2.options;

            boolean padResetExtendo = gamepad2.dpad_up && gamepad2.options;


            // Update the speed
            if (padSlowMode) {
                speed = .35;
            } else if (padFastMode) {
                speed = 1.5;
            } else {
                speed = 0.3;//.8; // prev 0.8
            }
            // especially in driver practice, imu drifts eventually
            // this lets them reset just in case
            if (padResetPose) {
                if (!(PoseStorage.currentTeam == PoseStorage.Team.BLUE)) { // Team is declared and saved there for auto
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90.0));
                } else {
                    drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
                }
                targetHeading = drive.pose.heading;
                gamepad1.rumbleBlips(1); // tell the driver it succeeded
            }
            // Second layer
            if (pad1ExtraSettings) {
                if (pad1ExToggleFieldCentric) {
                    fieldCentric = !fieldCentric;
                    if (fieldCentric) {
                        gamepad1.rumbleBlips(2);
                    } else {
                        gamepad1.rumbleBlips(1);
                    }
                }

                if (pad1ExTeamSwitch) {
                    if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                        gamepad1.rumbleBlips(1);
                        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                    } else {
                        gamepad1.rumbleBlips(2);
                        PoseStorage.currentTeam = PoseStorage.Team.RED;
                    }
                }
            }

            // Field Centric

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * speed,
                    -gamepad1.left_stick_x * speed
            );

            //Pose2d poseEstimate = drive.pose;
            double rotationAmount = -drive.pose.heading.log(); // Rotation2d.log() makes it into a double in radians.
            if (fieldCentric && !padCameraAutoAim) {
                if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) { // Depending on which side we're on, the color changes
                    rotationAmount = rotationAmount - Math.toRadians(90);
                } else {
                    rotationAmount = rotationAmount + Math.toRadians(90);

                }
                input = Rotation2d.fromDouble(rotationAmount).times(new Vector2d(input.x, input.y)); // magic courtesy of https://github.com/acmerobotics/road-runner/issues/90#issuecomment-1722674965
            }
            Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

            if (drivingEnabled) {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    drive.setDrivePowers(
                            new PoseVelocity2d(
                                    new Vector2d(
                                            input.x,
                                            input.y
                                    ),
                                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed
                            )
                    );
                    targetHeading = drive.pose.heading;
                    timeSinceDriverTurned.reset();
                } else {
                    // Set the target heading for the heading controller to our desired angle
                    if (Math.sqrt(Math.pow(controllerHeading.x, 2.0) + Math.pow(controllerHeading.y, 2.0)) > 0.4) {
                        // Cast the angle based on the angleCast of the joystick as a heading
                        if (PoseStorage.currentTeam == PoseStorage.Team.BLUE) {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(-90));
                        } else {
                            targetHeading = controllerHeading.angleCast().plus(Math.toRadians(90));
                        }
                    }

                    joystickHeadingController.targetPosition = targetHeading.toDouble();

                    double headingInput;
                    // Set the desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    if (timeSinceDriverTurned.milliseconds() > 250) {
                        headingInput = (joystickHeadingController.update(drive.pose.heading.log())
                                * MecanumDrive.PARAMS.kV
                                * MecanumDrive.PARAMS.trackWidthTicks);
                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(
                                                input.x,
                                                input.y
                                        ),
                                        headingInput
                                )
                        );

                    } else {
                        headingInput = 0;
                        targetHeading = drive.pose.heading;
                        drive.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(
                                                input.x,
                                                input.y
                                        ),
                                        headingInput
                                )
                        );
                    }

                }
            }


            // LIFT CONTROL/FSM



            // Slide (Manual)
            // TODO: abstract this?
            if (motorControl.deposit.getTargetPosition() > 1600 && padDepositControl > 0) {
                motorControl.deposit.setTargetPosition(1600);

            } else if (motorControl.deposit.getTargetPosition() <= 20 && padDepositControl < 0 && !padForceDown) {
                motorControl.deposit.findZero();
                motorControl.deposit.setTargetPosition(20);

            } else {
                motorControl.deposit.setTargetPosition(motorControl.deposit.getTargetPosition() + (padDepositControl * padSlideControlMultiplier));
            }
            if (motorControl.extendo.getTargetPosition() > 1530 && padExtendoControl > 0) {
                motorControl.extendo.setTargetPosition(1530);

            } else if (motorControl.extendo.getTargetPosition() <= 20 && padExtendoControl < 0 && !padForceDown) {
                motorControl.extendo.findZero();
                motorControl.extendo.setTargetPosition(20);

            } else {
                motorControl.extendo.setTargetPosition(motorControl.extendo.getTargetPosition() + (padExtendoControl * padSlideControlMultiplier));
            }

            if (padDepositClawToggle) {
                //motorControl.depositClaw.toggle();
            }
            if (padExtendoClawToggle) {
                //motorControl.extendoClaw.toggle();
            }

            if (padArmToggle) {
                motorControl.sArm.toggle();
            }
            if (padArmUpFull) {
                motorControl.sArm.setPosition(1.0);
            }



            if (padHighPreset) {
                motorControl.deposit.setTargetPosition(1600);
            }
            if (padMidPreset) {
                motorControl.extendo.setTargetPosition(1530);
            }
            if (padLowPreset) {
                motorControl.deposit.setTargetPosition(20);
                motorControl.extendo.setTargetPosition(20);
            }




            double colorAlpha = 0;
            double pad2rumble;

            // rumble the gunner controller based on the claw color sensor
            if (colorAlpha > 200) {// && !motorControl.extendoClaw.closed) {
                pad2rumble = Math.log10(colorAlpha) / 6;
            } else {
                pad2rumble = 0;
            }
            gamepad2.rumble(pad2rumble, pad2rumble, Gamepad.RUMBLE_DURATION_CONTINUOUS);

            // update RR, update motor controllers


            // TELEMETRY

            TelemetryPacket packet = new TelemetryPacket();
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

            updateAsync(packet);
            drive.updatePoseEstimate();
            motorControl.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            double loopTimeMs = loopTime.milliseconds();
            loopTimeAvg.add(loopTimeMs);
            while (loopTimeAvg.size() > 1000) {
                loopTimeAvg.removeFirst();
            }

            if (showPoseTelemetry) {
                telemetry.addLine("--- Pose ---");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading.log());
            }
            if (showLoopTimes) {
                telemetry.addLine("--- Loop Times ---");
                telemetry.addData("loopTimeMs", loopTimeMs);
                telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
                telemetry.addData("LoopAverage ", loopTimeAvg.stream().reduce(0.0,Double::sum) / loopTimeAvg.size());
                telemetry.addData("msBeforeUpdate",loopTimeBeforeUpdate);
                telemetry.addData("msAfterUpdate",loopTimeAfterUpdate);
            }

            if (showMotorTelemetry) {
                telemetry.addLine("--- Motors ---");
                telemetry.addData("extendoTarget", motorControl.extendo.getTargetPosition());
                telemetry.addData("extendoPosition", motorControl.extendo.getPosition());
                telemetry.addData("depositTarget", motorControl.deposit.getTargetPosition());
                telemetry.addData("depositPosition", motorControl.deposit.getPosition());
                //telemetry.addData("extendoClawPos", motorControl.extendoClaw.getPosition());
                //telemetry.addData("depositClawPos", motorControl.depositClaw.getPosition());
            }


            if (showStateTelemetry) {
                telemetry.addLine("--- State Machine ---");
                telemetry.addData("actions",runningActions);
            }
            loopTimeBeforeUpdate = loopTime.milliseconds();
            telemetry.update();
            loopTimeAfterUpdate = loopTime.milliseconds();
        }
    }


        // TODO: probably not needed, just make a normal action
        interface input {
            boolean isPressed();
        }
        Action waitForInput (input input){
            return telemetryPacket -> input.isPressed();
        }


}

