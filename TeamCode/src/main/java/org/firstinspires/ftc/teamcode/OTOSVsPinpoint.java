package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

@TeleOp
public class OTOSVsPinpoint extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        SparkFunOTOSDrive OTOSDrive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
        ElapsedTime pinpointTime = new ElapsedTime();
        ElapsedTime OTOSTime = new ElapsedTime();
        LinkedList<Double> pinpointTimeAvg = new LinkedList<>();
        LinkedList<Double> OTOSTimeAvg = new LinkedList<>();

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            pinpointTime.reset();
            drive.updatePoseEstimate();
            double pinpointTimeMS = pinpointTime.milliseconds();
            pinpointTimeAvg.add(pinpointTimeMS);
            pinpointTimeAvg = cropToLength1000(pinpointTimeAvg);
            OTOSTime.reset();
            OTOSDrive.updatePoseEstimate();
            double otosTimeMS = OTOSTime.milliseconds();
            OTOSTimeAvg.add(otosTimeMS);
            OTOSTimeAvg = cropToLength1000(OTOSTimeAvg);

            telemetry.addLine("PINPOINT");
            telemetry.addData("pinpoint x", drive.pose.position.x);
            telemetry.addData("pinpoint y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("pinpoint loop time", pinpointTimeMS);
            telemetry.addData("pinpoint loop time (average over 1k)", pinpointTimeAvg.stream().reduce(Double::sum).orElse(0.0) / pinpointTimeAvg.size());
            telemetry.addLine("OTOS");
            telemetry.addData("otos x", OTOSDrive.pose.position.x);
            telemetry.addData("otos y", OTOSDrive.pose.position.y);
            telemetry.addData("otos heading (deg)", Math.toDegrees(OTOSDrive.pose.heading.toDouble()));
            telemetry.addData("otos loop time", otosTimeMS);
            telemetry.addData("otos loop time (average over 1k)",OTOSTimeAvg.stream().reduce(Double::sum).orElse(0.0) / OTOSTimeAvg.size());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            packet.fieldOverlay().setStroke("red");
            Drawing.drawRobot(packet.fieldOverlay(), OTOSDrive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    LinkedList<Double> cropToLength1000(LinkedList<Double> input) {
        while (input.size() > 1000) {
            input.removeFirst();
        }
        return input;
    }
}
