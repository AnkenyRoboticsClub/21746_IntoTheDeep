package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double startX = 14;
        double startY = -63;
        double topY = 55+startY;
        double firstX = 31+startX;
        double secondX = 42+startX;
        double wallX = 47+startX;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(startX, startY, Math.toRadians(-90)))
                        //.forward(0.1)
                        /*.forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))*/
                        .lineToLinearHeading(new Pose2d(20+startX, 30+startY, Math.toRadians(-90)))
                        .splineToLinearHeading(new Pose2d(20+startX, topY, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(firstX, topY, Math.toRadians(-90)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(firstX, 5+startY, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(firstX, topY, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(secondX, topY, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(secondX, 5+startY, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(36+startX, topY, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(wallX, topY, Math.toRadians(180)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(wallX, 10+startY, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(20+startX, 5+startY, Math.toRadians(180)), Math.toRadians(180))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}