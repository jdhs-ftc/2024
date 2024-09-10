package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@TeleOp(name = "PIDToPointTest")
public final class PIDToPointTest extends LinearOpMode {
    public static double DISTANCE = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        new SequentialAction(
                        drive.pidToPointAction(new Pose2d(DISTANCE, 0, 0)),
                        //drive.pidToPointAction(new Pose2d(DISTANCE, DISTANCE, 0)),
                        //drive.pidToPointAction(new Pose2d(0, DISTANCE, 0)),
                        drive.pidToPointAction(new Pose2d(0, 0, 0))
                        )
                );
            }
        }
    }
}
