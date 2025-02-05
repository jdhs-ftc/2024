package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PinpointExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap,startPose);

        Action traj = drive.actionBuilder(startPose)
                        .splineTo(new Vector2d(30,30),Math.toRadians(0))
                        .build();


        waitForStart();

        Actions.runBlocking(traj);
    }
}
