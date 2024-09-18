package com.example;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800,120);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -63, Math.toRadians(-90)))
                        .setReversed(true)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(11, -33), Math.toRadians(90)) // go to sub
                        .waitSeconds(0.25)
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(40,-50,Math.toRadians(90)),Math.toRadians(270)) // go to line up point
                        // vision align should go here
                        .splineToSplineHeading(new Pose2d(40,-62,Math.toRadians(90)),Math.toRadians(270)) // go to hp
                        .waitSeconds(0.25)
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(9,-33,Math.toRadians(-90)),Math.toRadians(90)) //back to sub
                        .waitSeconds(0.25)
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(40,-50,Math.toRadians(90)),Math.toRadians(270)) // go to line up point
                        // vision align should go here
                        .splineToSplineHeading(new Pose2d(40,-62,Math.toRadians(90)),Math.toRadians(270)) // go to hp
                        .waitSeconds(0.25)
                        .setTangent(Math.toRadians(135))
                        .splineToSplineHeading(new Pose2d(7,-33,Math.toRadians(-90)),Math.toRadians(90)) //back to sub
                        .waitSeconds(0.25)
                // park in ascend zone 1
                        .setTangent(Math.toRadians(270))
                        .splineTo(new Vector2d(23.1, -10.2), Math.toRadians(90))

                /* get preset samples??
                        .setTangent(Math.toRadians(315))
                        .splineToSplineHeading(new Pose2d(35.3, -24.12,Math.toRadians(0)),Math.toRadians(90))
                        .waitSeconds(0.25)
                        .setTangent(Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(40,-60,Math.toRadians(270.00000001)),Math.toRadians(270))

                 */




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}