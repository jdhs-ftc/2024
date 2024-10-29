package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.ActionHelpers
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Suppress("unused")
@Autonomous(name = "Auto 0+3", group = "Auto", preselectTeleOp = "Teleop Field Centric")
class Auto0_3 : LinearOpMode() {
    override fun runOpMode() {
        val beginPose = Pose2d(6.625, -63.0, toRadians(90.0))

        val drive = PinpointDrive(hardwareMap, beginPose)
        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)

        val traj = drive.actionBuilder(beginPose)
            .afterTime(0.1,motorActions.deposit.setTargetPosition(1521.0))
            .setReversed(true)
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(11.0, -33.0), toRadians(90.0)) // go to sub
            .stopAndAdd(SequentialAction( // deposit at sub
                motorActions.deposit.setTargetPosition(1150.0),
                SleepAction(0.3),
                motorActions.depositClaw.open()
            ))
            .setTangent(toRadians(315.0))
            .splineToConstantHeading(
                Vector2d(47.0, -42.3),
                toRadians(90.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction (
                    motorActions.extendoCycle()
                )
            )
            .turnTo(toRadians(-90.0))
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(500.0),
                    SleepAction(0.5),
                    motorActions.extendoArm.moveDown(),
                    motorActions.extendoClaw.open(),
                    SleepAction(0.3),
                    motorActions.extendoArm.moveUp(),
                    motorActions.extendo.moveDown()
                )
            )
            .splineToSplineHeading(
                Pose2d(57.0, -42.3, toRadians(90.0)),
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction (
                    motorActions.extendoCycle()
                )
            )
            .turnTo(toRadians(-90.0000000001))
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(500.0),
                    SleepAction(0.5),
                    motorActions.extendoArm.moveDown(),
                    motorActions.extendoClaw.open(),
                    SleepAction(0.3),
                    motorActions.extendoArm.moveUp(),
                    motorActions.extendo.moveDown()
                )
            )
            .stopAndAdd(motorActions.depositMoveWall())
            .splineToSplineHeading(Pose2d(40.0, -50.0, toRadians(-90.0)), toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToSplineHeading(Pose2d(40.0, -63.0, toRadians(-90.0)), toRadians(270.0)) // go to hp
            .stopAndAdd(
                SequentialAction(
                motorActions.depositPickupWall(),
                SleepAction(0.5),
                motorActions.deposit.setTargetPosition(1521.0)
                )

            )
            .setTangent(toRadians(135.0))
            .splineToSplineHeading(Pose2d(9.0, -33.0, toRadians(90.0)), toRadians(90.0)) //back to sub
            .stopAndAdd(SequentialAction( // deposit at sub
                motorActions.deposit.setTargetPosition(1150.0),
                SleepAction(0.3),
                motorActions.depositClaw.open()
            ))
            .setTangent(toRadians(-90.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToSplineHeading(Pose2d(40.0, -50.0, toRadians(-90.0)), toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToSplineHeading(Pose2d(40.0, -63.0, toRadians(-90.0)), toRadians(270.0)) // go to hp
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositPickupWall(),
                    SleepAction(0.5),
                    motorActions.deposit.setTargetPosition(1521.0)
                )

            )
            .setTangent(toRadians(135.0))
            .splineToSplineHeading(Pose2d(6.0, -33.0, toRadians(90.0)), toRadians(90.0)) //back to sub
            .stopAndAdd(SequentialAction( // deposit at sub
                motorActions.deposit.setTargetPosition(1150.0),
                SleepAction(0.3),
                motorActions.depositClaw.open()
            ))

            .build()

        motorControl.depositClaw.close()

        waitForStart()

        runBlocking(
            ActionHelpers.RaceParallelCommand(
                traj,
                motorActions.update()
            )
        )

        PoseStorage.currentPose = drive.pose
        PoseStorage.poseUpdatedTime = System.currentTimeMillis()
    }
}
