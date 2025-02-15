package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(33.3, 61.8, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(52, 50), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(52, 47.2), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(50, 47.2), Math.toRadians(-100))
                .strafeToLinearHeading(new Vector2d(52, 50), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(62, 47.2), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(60, 47.2), Math.toRadians(-100))
                .strafeToLinearHeading(new Vector2d(52, 50), Math.toRadians(230))
                .strafeToLinearHeading(new Vector2d(54.5, 35.5), Math.toRadians(-30))
                .strafeToLinearHeading(new Vector2d(52, 50), Math.toRadians(230))
                .strafeToSplineHeading(new Vector2d(45, 15), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(28, -8), Math.toRadians(180))

                .strafeToSplineHeading(new Vector2d(45, 15), Math.toRadians(230))
                .splineToConstantHeading(new Vector2d(52, 50), Math.toRadians(230))




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}