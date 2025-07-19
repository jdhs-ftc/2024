package org.firstinspires.ftc.teamcode.auto

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
import org.firstinspires.ftc.teamcode.OctoQuadDrive
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.PoseStorage.Team
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Autonomous(preselectTeleOp = "00 Teleop Field Centric")
class AutoLeft : LinearOpMode() {
    override fun runOpMode() {
        val xPos = 11.675
        val scoreXPos = 15.75
        val hpPose = Pose2d(xPos, -48.5, toRadians(-90.0))
        val startPose = Pose2d(29.7, -61.5, toRadians(90.0))
        val startPoseMirrored = Pose2d(-startPose.position.x,startPose.position.y,startPose.heading.log())
        val depositY1 = 2.5
        val depositY2 = 0.0
        val depositY3 = -2.5
        val depositY4 = -5.0

        val intakeY1 = -15.0
        val intakeY2 = -5.0
        val intakeY3 = 1.0


        val drive = OctoQuadDrive(hardwareMap, startPoseMirrored)
        drive.writePose(startPoseMirrored)

        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)
        motorControl.depositClaw.close()


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


        val traj = drive.actionBuilderPathCRIMirrored(startPose)
            .setTangent(toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFarNoGrab())
            .splineToSplineHeading(
                Pose2d(xPos - 1.25, -30.0, toRadians(210.0)),
                toRadians(90.0)
            )

            .splineToConstantHeading(
                Vector2d(xPos - 1.25, -7.0),
                toRadians(90.0)
            )
            .splineToSplineHeading(Pose2d(scoreXPos, depositY1, toRadians(180.0)), toRadians(80.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(-130.0))
            //.splineToConstantHeading(Vector2d(xPos, -20.), toRadians(-90.0))
            .afterTime(0.7, motorActions.intakePreset())
            .splineToSplineHeading(Pose2d(xPos, intakeY1, toRadians(180.0)), toRadians(-90.0))
            .stopAndAdd(motorActions.intakePresetFinish())
            // third preset
            .setTangent(toRadians(-90.0))
            .afterTime(0.5, motorActions.intakeAutoHpEject(drive))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositPickupWall(),
                )
            )
            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFarNoGrab())
            .splineToConstantHeading(Vector2d(scoreXPos + 0.2, depositY2), toRadians(80.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(-130.0))
            // second preset
            .splineToConstantHeading(Vector2d(xPos, intakeY2), toRadians(-90.0))
            .stopAndAddHold(motorActions.intakePreset())
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(xPos, -24.0), toRadians(-90.0))
            .afterTime(0.5, motorActions.intakeAutoHpEject(drive))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositPickupWall(),
                )
            )

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFarNoGrab())
            .splineToConstantHeading(Vector2d(scoreXPos + 0.4, depositY3), toRadians(80.0))
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositScoreChamberFar()
                )
            )
            .setTangent(toRadians(-130.0))
            // first preset
            .afterTime(0.0, motorActions.intakePreset(300.0))
            .splineToConstantHeading(Vector2d(xPos, intakeY3), toRadians(-90.0))
            .stopAndAddHold(motorActions.intakePresetFinish())
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(xPos, -24.0), toRadians(-90.0))
            .afterTime(0.5, motorActions.intakeAutoHpEject(drive))
            .splineToSplineHeading(hpPose, toRadians(-90.0))
            // intake reverse + grab hp
            .stopAndAddHold(
                SequentialAction(
                    motorActions.depositPickupWall(),
                )
            )

            .setTangent(toRadians(90.0))
            .splineToSplineHeading(Pose2d(xPos, -26.0, toRadians(180.0)), toRadians(90.0))
            .afterTime(0.0, motorActions.depositMoveChamberFarNoGrab())
            .splineToConstantHeading(Vector2d(scoreXPos + 0.6, depositY4), toRadians(80.0))
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
            drive.writePose(startPoseMirrored)
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

