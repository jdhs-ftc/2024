package com.example

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.lang.Math.toRadians

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        System.setProperty("sun.java2d.opengl", "true")
        val meepMeep = MeepMeep(800, 120)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.0, 50.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .build()

        val motorActions = MotorActions()

        val beginPose = Pose2d(6.625, -63.0, toRadians(90.0))

        val humanPlayerLineUp = Vector2d(36.0, -50.0)
        val humanPlayerVec = Vector2d(36.0, -64.1) // -64

        val specimenDepositY = -32.0 // prev 33
        myBot.runAction(myBot.drive.actionBuilder(beginPose) // TODO THIS IS CAUSE OF ANY ISSUES
//            .afterTime(0.1, motorActions.deposit.setTargetPosition(300.0))
//            .afterTime(0.11, motorActions.depositMoveChamber())
            .setReversed(true)
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(
                Vector2d(7.0, specimenDepositY),
                toRadians(90.0)
            ) // go to sub prev 14 (prev 11) TODO CHANGE EVER
            .stopAndAdd(
                SequentialAction(
//                    motorActions.depositScoreChamber(),
                    SleepAction(0.4), // prev 0.4
//                    motorActions.depositClaw.open(),
                    SleepAction(0.1),
                    //motorActions.depositArm.moveDown()
                )
            )
            .setTangent(toRadians(270.0))
//            .afterTime(0.5, InstantAction { motorActions.depositArm.threeArm.position = 0.40 })
            .splineToConstantHeading(
                Vector2d(47.1, -48.5), // 47.1 -45.5 with back of robot in HP zone
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
//                    motorActions.extendo.setTargetPosition(600.0),
                    SleepAction(0.1), // 0.2
                    motorActions.extendoCycle(),
                )
            )
            // TODO CYCLE
            .afterTime(
                0.0,
                SequentialAction(
//                    motorActions.extendoArm.moveFullUp(),
                    SleepAction(0.1),
//                    motorActions.transferFull(
//                        SleepAction(
//                            0.5
//                        )
//                    ),
                    SleepAction(0.8),
//                    motorActions.depositClaw.open()
                )
            )
            .setTangent(0.0)
            .splineToConstantHeading(
                Vector2d(55.0, -48.5), // prev 56.5 -42.3
                toRadians(0.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    SleepAction(0.2),
//                    motorActions.extendo.setTargetPosition(650.0),
                    motorActions.extendoCycle()
                )
            )
            .afterTime(
                0.0,
                SequentialAction(
//                    motorActions.transferFull(),
                    SleepAction(0.8),
//                    motorActions.depositClaw.open(),
                )
            )
            .turnTo(toRadians(45.0))
            /*.setTangent(0.0)
            .splineToConstantHeading(
                Vector2d(63.0, -48.5), // 47.1 -45.5 with back of robot in HP zone
                toRadians(0.0)
            )*/
            // grab
            .stopAndAdd(
                SequentialAction(
//                    motorActions.extendo.setTargetPosition(650.0),
                    SleepAction(0.2),
                    motorActions.extendoCycle()
                )
            )
            // TODO CYCLE
            .stopAndAdd( // simultaneous
                SequentialAction(
//                    motorActions.transferFull(),
                    SleepAction(0.8), // 0.8
//                    motorActions.depositClaw.open(),
//                    motorActions.extendoArm.moveFullUp(),
//                    motorActions.extendo.moveDown(),

                    )
            )
            .setTangent(toRadians(180.0))
            .afterTime(0.5, motorActions.depositMoveWall())
            .splineToSplineHeading(Pose2d(humanPlayerLineUp, toRadians(90.0)) , toRadians(270.0)) // go to line up point
            // vision align should go here
            .splineToConstantHeading(humanPlayerVec, toRadians(270.0)) // go to hp 35.0 -62.5
            .stopAndAdd(
                SequentialAction(
//                    motorActions.extendo.moveDown(),
                    motorActions.depositPickupWall(),
                    SleepAction(0.3),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(135.0))
            .splineToConstantHeading(Vector2d(4.0, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
//                    motorActions.depositClaw.open()
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
            .setTangent(toRadians(135.0))
            .splineToConstantHeading(Vector2d(1.0, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
//                    motorActions.depositClaw.open(),
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
            .setTangent(toRadians(135.0))
            .splineToConstantHeading(
                Vector2d(-1.0, specimenDepositY),
                toRadians(90.0)
            ) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamber(),
                    SleepAction(0.4),
//                    motorActions.depositClaw.open(),
//                    motorActions.extendoArm.moveUp()
                )
            )
            .waitSeconds(5.0)

            /*
            // park in ascend zone 1 (kinda bad)
            .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(-33.3, -35.1,Math.toRadians(90)),Math.toRadians(135))
            .splineToSplineHeading(new Pose2d(-23.1, -10.2,Math.toRadians(0)), Math.toRadians(0))

             */
            /* get preset samples??
                    .setTangent(Math.toRadians(315))
                    .splineToSplineHeading(new Pose2d(35.3, -24.12,Math.toRadians(0)),Math.toRadians(90))
                    .waitSeconds(0.25)
                    .setTangent(Math.toRadians(270))
                    .splineToSplineHeading(new Pose2d(40,-60,Math.toRadians(270.00000001)),Math.toRadians(270))

             */
            /* get preset samples attempt 2??
            .splineToSplineHeading(new Pose2d(40, -45, Math.toRadians(75)), Math.toRadians(0))// go to midpoint between hp and preset, facing 1st preset
            .waitSeconds(1) // extend extendo and grab
            .turnTo(Math.toRadians(315)) // turn to hp
            .waitSeconds(1) // give to hp
            .turnTo(Math.toRadians(45)) // turn to second preset
            .waitSeconds(1) // extend extendo and grab
            .turnTo(Math.toRadians(315)) // turn to hp
            .waitSeconds(1) // give to hp
            .turnTo(Math.toRadians(30))
            .waitSeconds(1) // extend extendo and grab
            .turnTo(Math.toRadians(315)) // turn to hp
            .waitSeconds(1) // give to hp
             */


            .build()

        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}