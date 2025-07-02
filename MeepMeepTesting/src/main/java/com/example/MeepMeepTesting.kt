package com.example

import com.acmerobotics.roadrunner.Pose2d
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

        val hpPose = Pose2d(13.0,-50.0,toRadians(-90.0))


        val beginPose2 = Pose2d(29.0, -60.0, toRadians(90.0))
        myBot.runAction(
            myBot.drive.actionBuilder(beginPose2)
                .splineToSplineHeading(Pose2d(13.0, -24.0, toRadians(180.0)), toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -3.0, toRadians(180.0)), toRadians(90.0))
                // score
                // grab first preset
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // intake reverse +  grab hp
                .waitSeconds(0.25)

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -12.0, toRadians(180.0)), toRadians(90.0))
                // score
                // grab second preset
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // intake reverse + grab hp
                .waitSeconds(0.25)

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -12.0, toRadians(180.0)), toRadians(90.0))
                // score
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(Pose2d(13.0, -22.0, toRadians(180.0)), toRadians(-90.0))
                .waitSeconds(0.1)
                .setTangent(toRadians(-90.0))
                // third preset
                .waitSeconds(0.25)
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // intake reverse + grab hp
                .waitSeconds(0.25)

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -3.0, toRadians(180.0)), toRadians(90.0))
                // score
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // grab hp
                .waitSeconds(0.25)

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -3.0, toRadians(180.0)), toRadians(90.0))
                // score
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // grab hp
                .waitSeconds(0.25)


                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -3.0, toRadians(180.0)), toRadians(90.0))
                // score
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // grab hp
                .waitSeconds(0.25)

                .setTangent(toRadians(90.0))
                .splineToSplineHeading(Pose2d(13.0, -3.0, toRadians(180.0)), toRadians(90.0))
                // score
                .waitSeconds(0.25)
                .setTangent(toRadians(-90.0))
                .splineToSplineHeading(hpPose, toRadians(-90.0))
                // grab hp
                .waitSeconds(0.25)


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