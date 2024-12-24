package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

// 0+3 plus one in sub working? Kinda dubious
// 0+4 maybe if pushed?
// 0+5 impossible :(((
@Suppress("unused")
@Autonomous(name = "Auto 4 attempt 2", group = "Auto", preselectTeleOp = "Teleop Field Centric")
class Auto4part2 : LinearOpMode() {
    // todo don't slam into wall
    // some weird stuff happens sometimes, is wall slam dcing??
    override fun runOpMode() {
        val beginPose = Pose2d(6.625, -63.0, toRadians(90.0))
        val team = PoseStorage.Team.RED

        val drive = PinpointDrive(hardwareMap, beginPose)
        val motorControl = MotorControl(hardwareMap)
        val motorActions = MotorActions(motorControl)
        val humanPlayerLineUp = Vector2d(36.0, -50.0)
        val humanPlayerVec = Vector2d(36.0, -63.5) // -64.1

        val specimenDepositY = -32.0 // prev 33

        val traj = drive.actionBuilderPath(beginPose) // TODO THIS IS CAUSE OF ANY ISSUES
            .afterTime(0.1, motorActions.deposit.setTargetPosition(300.0))
            .afterTime(0.11, motorActions.depositMoveChamber())
            .setReversed(true)
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(
                Vector2d(7.0, specimenDepositY),
                toRadians(90.0)
            ) // go to sub prev 14 (prev 11) TODO CHANGE EVER
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4), // prev 0.4
                    motorActions.depositClaw.open(),
                    SleepAction(0.1),
                    //motorActions.depositArm.moveDown()
                )
            )
            .setTangent(toRadians(270.0))
            .afterTime(0.5, InstantAction { motorActions.depositArm.threeArm.position = 0.40 })
            .splineToConstantHeading(
                Vector2d(46.5, -48.5), // 47.1 -45.5 with back of robot in HP zone
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(600.0),
                    SleepAction(0.1), // 0.2
                    motorActions.extendoCycle(),
                )
            )
            // TODO CYCLE
            .afterTime(
                0.0,
                SequentialAction(
                    motorActions.extendoArm.moveFullUp(),
                    SleepAction(0.1),
                    motorActions.transferFull(
                        SleepAction(
                            0.5
                        )
                    ),
                    SleepAction(0.8),
                    motorActions.depositClaw.open()
                )
            )
            .setTangent(0.0)
            .splineToConstantHeading(
                Vector2d(54.0, -48.5), // prev 55 -48.5 // prev 56.5 -42.3
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    SleepAction(0.2),
                    motorActions.extendo.setTargetPosition(670.0), // 650
                    motorActions.extendoCycle()
                )
            )
            .afterTime(
                0.0,
                SequentialAction(
                    motorActions.transferFull(),
                    SleepAction(0.8),
                    motorActions.depositClaw.open(),
                )
            )
            .turnTo(toRadians(65.0))
            // grab
            .stopAndAdd(
                SequentialAction(
                    SleepAction(0.2), // wait for pass to finish
                    motorActions.extendo.setTargetPosition(900.0), // 950
                    SleepAction(0.2),
                    motorActions.extendoCycle()
                )
            )
            // TODO CYCLE
            .stopAndAdd( // simultaneous
                SequentialAction(
                    motorActions.transferFull(),
                    SleepAction(0.8), // 0.8
                    motorActions.depositClaw.open(),
                    motorActions.extendoArm.moveFullUp(),
                    motorActions.extendo.moveDown(),

                )
            )
            .setTangent(toRadians(180.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToSplineHeading(
                Pose2d(humanPlayerLineUp, toRadians(90.0)),
                toRadians(270.0)
            ) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec, toRadians(270.0))//, TranslationalVelConstraint(10.0)) // go to hp 35.0 -62.5
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.moveDown(),
                    motorActions.depositPickupWall(),
                    SleepAction(0.3),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(Vector2d(4.0, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
                    motorActions.depositClaw.open()
                )
            )
            .setTangent(toRadians(-90.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToConstantHeading(humanPlayerLineUp, toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec, toRadians(270.0)) // go to hp
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositPickupWall(),
                    SleepAction(0.3),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(Vector2d(1.0, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
                    motorActions.depositClaw.open(),
                )
            )
            .setTangent(toRadians(-90.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToConstantHeading(humanPlayerLineUp, toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec, toRadians(270.0)) // go to hp
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositPickupWall(),
                    SleepAction(0.3),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(
                Vector2d(-1.0, specimenDepositY),
                toRadians(90.0)
            ) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
                    motorActions.depositClaw.open(),
                )
            )
            .waitSeconds(5.0)



            .build()

        motorControl.depositClaw.close()

        waitForStart()

        runBlocking(
            RaceParallelAction(
                traj,
                motorActions.update()
            )
        )

        PoseStorage.currentPose = drive.pose
        PoseStorage.poseUpdatedTime = System.currentTimeMillis()
        PoseStorage.currentTeam = team
    }
}
