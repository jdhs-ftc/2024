package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
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
// 0+5 DONE!!!!!
@Suppress("unused")
@Autonomous(name = "Auto 4 attempt 2", group = "Auto", preselectTeleOp = "Teleop Field Centric")
class Auto4part2 : LinearOpMode() {
    lateinit var motorControl: MotorControl
    lateinit var motorActions: MotorActions
    // todo don't slam into wall
    // some weird stuff happens sometimes, is wall slam dcing??
    val openingTransferDelay: Action
        get() = motorActions.depositEncoder.waitForTransferRelease()


    override fun runOpMode() {
        val beginPose = Pose2d(6.625, -63.0, toRadians(90.0))
        val team = PoseStorage.Team.RED

        val drive = PinpointDrive(hardwareMap, beginPose)
        motorControl = MotorControl(hardwareMap)
        motorActions = MotorActions(motorControl)
        val humanPlayerLineUp = Vector2d(33.0, -62.0) // 36 -50
        val humanPlayerVec = Vector2d(33.0, -63.5) // -64.1

        val specimenDepositY = -32.0 // prev 33



        val traj = drive.actionBuilderPath(beginPose) // TODO THIS IS CAUSE OF ANY ISSUES
            .afterTime(0.1, motorActions.deposit.setTargetPosition(300.0))
            .afterTime(0.11, motorActions.depositMoveChamber())
            .setReversed(true)
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(
                Vector2d(10.5, specimenDepositY),
                toRadians(90.0)
            ) // go to sub prev 14 (prev 11) TODO CHANGE EVER
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.1), // prev 0.4
                    motorActions.depositClaw.open()
                    //motorActions.depositArm.moveDown()
                )
            )
            .setTangent(toRadians(270.0))
            .afterTime(0.5, InstantAction { motorActions.depositArm.threeArm.position = 0.40 })
            .splineToConstantHeading(
                Vector2d(46.0, -48.5), // 47.1 -45.5 with back of robot in HP zone
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(650.0),
                    motorActions.extendo.waitUntilFinished(),
                    motorActions.extendoCycle(SleepAction(0.4)),
                )
            )
            // TODO CYCLE
            .afterTime(
                0.0,
                SequentialAction(
                    motorActions.extendoArm.moveFullUp(),
                    motorActions.extendo.moveTransfer(),
                    SleepAction(0.2),
                    motorActions.transferFull(),
                    openingTransferDelay,
                    motorActions.depositClaw.open()
                )
            )
            .setTangent(0.0)
            .splineToConstantHeading(
                Vector2d(56.5, -48.5), // prev 55 -48.5 // prev 56.5 -42.3
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    SleepAction(0.2),
                    motorActions.extendo.setTargetPosition(650.0), // 650
                    motorActions.extendo.waitUntilFinished(),
                    motorActions.extendoCycle(),
                    motorActions.transferFull(), // 0.65
                    openingTransferDelay,
                    motorActions.depositClaw.open(),
                )
            )

            .turnTo(toRadians(65.0))
            // grab
            .stopAndAdd(
                SequentialAction(
                    //SleepAction(0.2), // wait for pass to finish
                    motorActions.extendo.setTargetPosition(825.0), // 800 // 850 // 900
                    motorActions.extendo.waitUntilFinished(),
                    motorActions.extendoClaw.close(),
                    motorActions.extendoArm.moveDown(),
                    SleepAction(0.1),
                    motorActions.extendoClaw.open(),
                    SleepAction(0.3),
                    motorActions.extendoGrabAndRaise(),
                    motorActions.transferFull(),
                )
            )
            .turnTo(toRadians(90.0))
            // TODO CYCLE
            .stopAndAdd( // simultaneous
                SequentialAction(

                    openingTransferDelay,
                    motorActions.depositClaw.open(),
                    motorActions.extendoArm.moveFullUp(),
                    motorActions.extendo.moveDown(),

                )
            )
            .setTangent(toRadians(180.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToSplineHeading(
                Pose2d(humanPlayerLineUp + Vector2d(2.5,0.0), toRadians(90.0)),
                toRadians(270.0)
            ) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec + Vector2d(2.5,0.0), toRadians(270.0))//, TranslationalVelConstraint(10.0)) // go to hp 35.0 -62.5
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.moveDown(),
                    motorActions.depositPickupWall(),
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(Vector2d(9.0, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.1),
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
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(Vector2d(7.5, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.1),
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
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(
                Vector2d(5.5, specimenDepositY),
                toRadians(90.0)
            ) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.1),
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
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(
                Vector2d(3.5, specimenDepositY),
                toRadians(90.0)
            ) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber()
                )
            ).setTangent(toRadians(-90.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToConstantHeading(humanPlayerLineUp, toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec, toRadians(270.0)) // go to hp
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositPickupWall(),
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0)) // 135
            .splineToConstantHeading(
                Vector2d(2.0, specimenDepositY),
                toRadians(90.0)
            ) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber()
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
