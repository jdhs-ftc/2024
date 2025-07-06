package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Autonomous(preselectTeleOp = "00 Teleop Field Centric")
class AutoRight: LinearOpMode() {
    override fun runOpMode() {
        val xPos = 12.0
        val hpPose = Pose2d(xPos, -50.0, toRadians(-90.0))
        val startPose = Pose2d(30.0, -62.0, toRadians(90.0))

        val drive = PinpointDrive(hardwareMap,startPose)
        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)


        val traj = drive.actionBuilderPath(startPose)
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -12.0, toRadians(180.0)), toRadians(90.0))
            // score
            .waitSeconds(0.25)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(Pose2d(xPos - 2, -22.0, toRadians(180.0)), toRadians(-90.0))
            .waitSeconds(0.1)
            .setTangent(toRadians(-90.0))
            // third preset
            .waitSeconds(0.25)
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .waitSeconds(0.25)
            /*


            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -12.0, toRadians(180.0)), toRadians(90.0))
            // score
            // grab second preset
            .waitSeconds(0.25)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .waitSeconds(0.25)

            .splineToSplineHeading(Pose2d(xPos, -24.0, toRadians(180.0)), toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -3.0, toRadians(180.0)), toRadians(90.0))
            // score
            // grab first preset
            .waitSeconds(0.25)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse +  grab hp
            .waitSeconds(0.25)

            .setTangent(toRadians(90.0))

             */
            .build()

        runBlocking(motorActions.depositArm.moveDown())

        while (opModeInInit()) {
            if (gamepad1.dpad_left) PoseStorage.currentTeam = Team.BLUE
            if (gamepad1.dpad_right) PoseStorage.currentTeam = Team.RED
            motorControl.update()
        }

        waitForStart()


        if (opModeIsActive() && !isStopRequested) {
            runBlocking(
                RaceParallelAction(
                    traj,
                    motorActions.update()
                )
            )
        }
    }
}