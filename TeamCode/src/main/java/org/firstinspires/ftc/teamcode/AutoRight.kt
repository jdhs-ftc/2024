package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Autonomous(preselectTeleOp = "00 Teleop Field Centric")
class AutoRight : LinearOpMode() {
    override fun runOpMode() {
        val xPos = 12.0
        val scoreXPos = 14.65
        val hpPose = Pose2d(xPos, -49.5, toRadians(-90.0))
        val startPose = Pose2d(29.7, -62.0, toRadians(90.0))
        val depositY1 = 0.0
        val depositY2 = -2.5
        val depositY3 = -5.0
        val depositY4 = -7.5


        val drive = OctoQuadDrive(hardwareMap, startPose)
        drive.writePose(startPose)

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
            ).endTrajectory()

             */


        fun TrajectoryActionBuilder.waitSecondsHold(seconds: Double) =
            this.stopAndAdd(SleepAction(seconds))


        val traj = drive.actionBuilderPath(startPose)
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(13.75, depositY1), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(-130.0))
            .splineToConstantHeading(Vector2d(xPos, -20.0), toRadians(-90.0))
            .splineToSplineHeading(Pose2d(xPos, -23.0, toRadians(180.0)), toRadians(-90.0))
            .stopAndAdd(motorActions.intakePreset())
            // third preset
            .setTangent(toRadians(-90.0))
            .afterTime(0.0, motorActions.depositMoveWall())
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.intakeAutoHpEject(),
                    motorActions.depositPickupWall(),
                )
            )
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(scoreXPos, depositY2), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(0.0))
            // second preset
            .splineToConstantHeading(Vector2d(xPos, -13.0), toRadians(-90.0))
            .stopAndAddHold(motorActions.intakePreset())
            .setTangent(toRadians(-90.0))
            .afterTime(0.0, motorActions.depositMoveWall())
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.intakeAutoHpEject(),
                    motorActions.depositPickupWall(),
                )
            )

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(scoreXPos, depositY3), toRadians(60.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(0.0))
            // first preset
            .splineToConstantHeading(Vector2d(xPos, -2.5), toRadians(-90.0))
            .stopAndAddHold(motorActions.intakePreset())
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(xPos, -24.0), toRadians(-90.0))
            .afterTime(0.0, motorActions.depositMoveWall())
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse +  grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.intakeAutoHpEject(),
                    motorActions.depositPickupWall(),
                )
            )

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFar())
            .splineToConstantHeading(Vector2d(scoreXPos, depositY4), toRadians(60.0))
            .stopAndAddHold( // 4th spec score
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )

            .build()

        motorControl.depositClaw.close()
        runBlocking(motorActions.depositArm.moveDown())

        while (opModeInInit()) {
            if (gamepad1.dpad_left) {
                PoseStorage.currentTeam = Team.BLUE
                motorControl.topLight.color = Color.BLUE
            }
            if (gamepad1.dpad_right) {
                PoseStorage.currentTeam = Team.RED
                motorControl.topLight.color = Color.RED
            }
            if (gamepad2.dpad_left) motorControl.depositClaw.open()
            if (gamepad2.dpad_right) motorControl.depositClaw.close()
            motorControl.update()
            drive.writePose(startPose)
        }

        waitForStart()

        val dash = FtcDashboard.getInstance()
        val c = Canvas()
        traj.preview(c)

        val motorActionsUpdate = motorActions.autoUpdate()

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

