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
                .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .build()

        val secondBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.6)
                .setDimensions(13.25, 18.0)
                .setColorScheme(ColorSchemeBlueLight())
                .build()

        val motorActions = MotorActions()

        val beginPose = Pose2d(-12.0, -60.0, toRadians(90.0))
        myBot.runAction(myBot.drive.actionBuilder(beginPose)
            .splineToConstantHeading(Vector2d(-12.0, -12.0), toRadians(90.0))
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-12.0, -60.0),toRadians(-90.0))
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(-12.0, -12.0), toRadians(90.0))
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-12.0, -60.0), toRadians(-90.0))
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(-12.0, -12.0), toRadians(90.0))
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-12.0, -60.0), toRadians(-90.0))
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(-12.0, -12.0), toRadians(90.0))
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-12.0, -60.0), toRadians(-90.0))
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(-12.0, -12.0), toRadians(90.0))
            .setTangent(toRadians(-90.0))
            .splineToConstantHeading(Vector2d(-12.0, -60.0), toRadians(-90.0))
            .build()
        )


        val beginPose2 = Pose2d(12.0, -60.0, toRadians(90.0))
        secondBot.runAction(
            secondBot.drive.actionBuilder(beginPose2)
                .splineToSplineHeading(Pose2d(12.0, -12.0, toRadians(0.0)), toRadians(90.0))
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(12.0, -60.0, toRadians(90.0)), toRadians(-90.0))

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(12.0, -12.0, toRadians(0.0)), toRadians(90.0))
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(12.0, -60.0, toRadians(90.0)), toRadians(-90.0))

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(12.0, -12.0, toRadians(0.0)), toRadians(90.0))
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(12.0, -60.0, toRadians(90.0)), toRadians(-90.0))

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(12.0, -12.0, toRadians(0.0)), toRadians(90.0))
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(12.0, -60.0, toRadians(90.0)), toRadians(-90.0))

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(12.0, -12.0, toRadians(0.0)), toRadians(90.0))
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(12.0, -60.0, toRadians(90.0)), toRadians(-90.0))
                .build()
        )

        val file = File("CRIField2dMeepMeep.png")
        val image = ImageIO.read(file)

        meepMeep.setBackground(image)
            .setDarkMode(false)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .addEntity(secondBot)
            .start()
    }
}