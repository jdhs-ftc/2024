package com.example

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.io.File
import java.lang.Math.toRadians
import javax.imageio.ImageIO

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        System.setProperty("sun.java2d.opengl", "true")
        val meepMeep = MeepMeep(800, 120)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80.0, 60.0, toRadians(180.0), toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .setColorScheme(ColorSchemeBlueLight())
                .build()

        val motorActions = MotorActions()


        val xPos = 11.675
        val scoreXPos = 14.65
        val hpPose = Pose2d(xPos, -49.5, toRadians(-90.0))
        val startPose = Pose2d(29.7, -62.0, toRadians(90.0))
        val depositY1 = 0.0
        val depositY2 = -2.5
        val depositY3 = -5.0
        val depositY4 = -7.5
        myBot.runAction(
            myBot.drive.actionBuilder(startPose)
                .setTangent(toRadians(90.0))
                .splineToSplineHeading(
                    Pose2d(xPos - 1.0, -30.0, toRadians(210.0)),
                    toRadians(90.0)
                )
                .splineToConstantHeading(
                    Vector2d(xPos - 1.0, -10.0),
                    toRadians(90.0)
                ).splineToSplineHeading(
                    Pose2d(scoreXPos, depositY1, toRadians(180.0)),
                    toRadians(80.0)
                )
                .build()
        )

        val file = File("CRIField2dMeepMeep.png")
        val image = ImageIO.read(file)

        meepMeep.setBackground(image)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}