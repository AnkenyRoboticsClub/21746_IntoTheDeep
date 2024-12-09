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
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double startX = 14;
        double startY = -63;
        double topY = 55+startY;
        double firstX = 31+startX;
        double secondX = 42+startX;
        double wallX = 47+startX;

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(startX, startY, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(20+startX, 30+startY), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(20+startX, topY, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(firstX, topY, Math.toRadians(-90)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(firstX, 5+startY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(firstX, topY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(secondX, topY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(secondX, 5+startY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(36+startX, topY), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(wallX, topY, Math.toRadians(180)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(wallX, 10+startY), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(20+startX, 5+startY, Math.toRadians(180)), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}