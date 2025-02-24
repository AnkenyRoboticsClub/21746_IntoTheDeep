package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double startX = 14;
        double startY = -63;
        double topY = 52+startY;
        double firstX = 31+startX;
        double secondX = 42+startX;
        double wallX = 47+startX;

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(startX, startY, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(20+startX, 30+startY), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(20+startX, topY, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(firstX, topY, Math.toRadians(-90)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(firstX, 5+startY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(firstX, topY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(secondX, topY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(secondX, 5+startY), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(36+startX, topY), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(wallX, topY, Math.toRadians(0)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(wallX, 10+startY), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(20+startX, 5+startY), Math.toRadians(0))
                .build());

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-33, -63, Math.toRadians(-90)))
                //score preload
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                //.strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                //get 1st off ground
                .strafeToLinearHeading(new Vector2d(-33, -40), Math.toRadians(160))
                .strafeToLinearHeading(new Vector2d(-34, -32), Math.toRadians(160))
                .strafeToLinearHeading(new Vector2d(-42, -31), Math.toRadians(160))
                //score 1st
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                //.strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                //get 2nd off ground
                .strafeToLinearHeading(new Vector2d(-44, -25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52, -25), Math.toRadians(180))
                //score 2nd
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                //.strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                //get 3rd off ground
                .strafeToLinearHeading(new Vector2d(-54, -25), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-62, -25), Math.toRadians(180))
                //score 3rd
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                //.strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                //park
                .strafeToLinearHeading(new Vector2d(-40, -20), Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(225)),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-25, -11), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}