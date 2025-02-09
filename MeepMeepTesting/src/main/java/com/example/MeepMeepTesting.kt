package com.example

import com.acmerobotics.roadrunner.Pose2d
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
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .build()

        val motorActions = MotorActions()

        val beginPose = Pose2d(25.0, -10.0, toRadians(180.0))

        val humanPlayerLineUp = Vector2d(36.0, -50.0)
        val humanPlayerVec = Vector2d(36.0, -64.1) // -64

        val specimenDepositY = -32.0 // prev 33
        myBot.runAction(myBot.drive.actionBuilder(beginPose)
            .setReversed(true)
            // RETRACT EXTENDO
            .splineToConstantHeading(Vector2d(45.0,-7.0),toRadians(300.0))
            // EXTEND EXTENDO
            .splineToSplineHeading(Pose2d(50.0,-40.0,toRadians(270.0)), toRadians(270.0))
            // DROP SAMPLE
            // RETRACT EXTENDO
            .setReversed(true)
            .splineToSplineHeading(Pose2d(45.0,-7.0,toRadians(180.0)), toRadians(135.0))
            .splineToConstantHeading(Vector2d(25.0,0.0),toRadians(180.0))
            .build()

        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}