package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Autonomous(preselectTeleOp = "00 Teleop Field Centric")
class AutoRight : LinearOpMode() {
    override fun runOpMode() {
        val xPos = 12.0
        val scoreXPos = 13.875
        val hpPose = Pose2d(xPos, -49.0, toRadians(-90.0))
        val startPose = Pose2d(30.0, -62.0, toRadians(90.0))

        val drive = PinpointDrive(hardwareMap, startPose)
        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)


        fun TrajectoryActionBuilder.stopAndAddHold(action: Action) = //this.stopAndAdd(action)

            this.afterDisp(
                1000.0, SequentialAction(
                    InstantAction { drive.makeTrajectoryWait = true },
                    action,
                    InstantAction { drive.makeTrajectoryWait = false }
                )
            ).endTrajectory()


        fun TrajectoryActionBuilder.waitSecondsHold(seconds: Double) =
            this.stopAndAdd(SleepAction(seconds))


        val traj = drive.actionBuilderPath(startPose)
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(13.75, -6.0), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    SleepAction(0.25),
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(-130.0))
            .splineToSplineHeading(Pose2d(xPos, -20.0, toRadians(180.0)), toRadians(-90.0))
            .splineToSplineHeading(Pose2d(7.0, -23.0, toRadians(180.0)), toRadians(-90.0))
            .waitSecondsHold(0.25)
            // third preset
            .setTangent(toRadians(-90.0))
            .afterTime(0.0, motorActions.depositMoveWall())
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(motorActions.depositPickupWall())

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(scoreXPos, -4.0), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    SleepAction(0.25),
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(0.0))
            // second preset
            .splineToConstantHeading(Vector2d(7.0, -12.0), toRadians(-90.0))
            .waitSecondsHold(0.1)
            .setTangent(toRadians(-90.0))
            .afterTime(0.0, motorActions.depositMoveWall())
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(motorActions.depositPickupWall())

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(scoreXPos, -2.0), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    SleepAction(0.25),
                    motorActions.depositScoreChamberFar()
                )
            )
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

        motorControl.depositClaw.close()
        runBlocking(motorActions.depositArm.moveDown())

        while (opModeInInit()) {
            if (gamepad1.dpad_left) PoseStorage.currentTeam = Team.BLUE
            if (gamepad1.dpad_right) PoseStorage.currentTeam = Team.RED
            if (gamepad2.dpad_left) motorControl.depositClaw.open()
            if (gamepad2.dpad_right) motorControl.depositClaw.close()
            motorControl.update()
        }

        waitForStart()

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        traj.preview(c)

        val motorActionsUpdate = motorActions.update()

        var b = true
        while (opModeIsActive() && !isStopRequested
            && b && !Thread.currentThread().isInterrupted
        ) {
            val p = TelemetryPacket()
            p.fieldOverlay().operations.addAll(c.operations)

            b = traj.run(p)
            motorActionsUpdate.run(p)
            PoseStorage.currentPose = drive.pose
            PoseStorage.poseUpdatedTime = System.currentTimeMillis()

            dash.sendTelemetryPacket(p)
        }
    }
}

