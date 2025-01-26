package com.example

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.lang.Math.toRadians

object AutoSample {
    @JvmStatic
    fun main(args: Array<String>) {
        System.setProperty("sun.java2d.opengl", "true")
        val meepMeep = MeepMeep(800, 120)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .build()

        val motorActions = MotorActions()

        val beginPose = Pose2d(-17.375, -63.0, toRadians(90.0))

        val humanPlayerLineUp = Vector2d(36.0, -50.0)
        val humanPlayerVec = Vector2d(36.0, -64.1) // -64

        val specimenDepositY = -32.0 // prev 33

        val basketPos = Pose2d(-60.0, -60.0, toRadians(45.0))

        myBot.runAction(myBot.drive.actionBuilder(beginPose)
            .setTangent(toRadians(90.0))
            .splineToLinearHeading(basketPos,toRadians(225.0 ))
            .setTangent(toRadians(45.0))
            .splineToConstantHeading(
                Vector2d(-46.5, -48.5), // 47.1 -45.5 with back of robot in HP zone
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
            .build()

        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}