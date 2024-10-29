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

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(12.0, -63.0, toRadians(90.0)))
                //.afterTime(0.1,motorActions.deposit.setTargetPosition(1521.0))
                .setReversed(true)
                .setTangent(toRadians(90.0))
                .splineToConstantHeading(Vector2d(11.0, -33.0), toRadians(90.0)) // go to sub
                .stopAndAdd(SequentialAction( // deposit at sub
                    //motorActions.deposit.setTargetPosition(1150.0),
                    SleepAction(0.3),
                    //motorActions.depositClaw.open()
                ))
                .setTangent(toRadians(315.0))
                .splineToConstantHeading(
                    Vector2d(48.3, -39.3),
                    toRadians(90.0)
                )
                .waitSeconds(1.0)
                .turnTo(toRadians(-90.0))
                .waitSeconds(1.0)
                .splineToSplineHeading(
                    Pose2d(57.0, -39.3, toRadians(90.0)),
                    toRadians(0.0)
                )
                .waitSeconds(1.0)
                .turnTo(toRadians(-90.0))
                .waitSeconds(1.0)


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