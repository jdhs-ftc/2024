package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
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
        val hpPose = Pose2d(xPos, -60.0, toRadians(-90.0))
        val startPose = Pose2d(30.0, -62.0, toRadians(90.0))

        val drive = PinpointDrive(hardwareMap,startPose)
        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)




        fun TrajectoryActionBuilder.stopAndAddHold(action: Action) = this.stopAndAdd(action)
/*
            this.afterDisp(
                1000.0, SequentialAction(
                    InstantAction { drive.makeTrajectoryWait = true },
                    action,
                    InstantAction { drive.makeTrajectoryWait = false }
                )
            )

 */

        fun TrajectoryActionBuilder.waitSecondsHold(seconds: Double) = this.stopAndAddHold(SleepAction(seconds))


        val traj = drive.actionBuilderPath(startPose)
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -24.0, toRadians(180.0)), toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -12.0, toRadians(180.0)), toRadians(90.0))
            // score
            .waitSecondsHold(0.25)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(Pose2d(xPos, -20.0, toRadians(180.0)), toRadians(-90.0))
            .splineToSplineHeading(Pose2d(7.0, -23.0, toRadians(180.0)), toRadians(-90.0))
            .waitSecondsHold(0.25)
            // third preset
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .waitSecondsHold(0.25)

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -12.0, toRadians(180.0)), toRadians(90.0))
            // score
            .waitSecondsHold(0.25)
            .setTangent(toRadians(0.0))
            // second preset
            .splineToConstantHeading(Vector2d(7.0, -12.0), toRadians(-90.0))
            .waitSecondsHold(0.1)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .waitSecondsHold(0.25)

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -24.0, toRadians(180.0)), toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -4.0, toRadians(180.0)), toRadians(90.0))
            // score
            .waitSecondsHold(0.25)
            .setTangent(toRadians(0.0))
            // first preset
            .splineToConstantHeading(Vector2d(7.0, -4.0), toRadians(-90.0))
            .waitSecondsHold(0.25)
            .setTangent(toRadians(-90.0))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse +  grab hp
            .waitSecondsHold(0.25)

            .setTangent(toRadians(90.0))

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

