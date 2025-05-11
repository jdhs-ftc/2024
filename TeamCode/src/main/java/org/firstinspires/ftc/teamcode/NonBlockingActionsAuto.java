package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public final class NonBlockingActionsAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Action traj = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                .build();

        FtcDashboard dash = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket p = new TelemetryPacket();
            traj.run(p);
            dash.sendTelemetryPacket(p);

            // non blocking
            // put whatever other update stuff you want here
        }


    }
}
