package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "AutoV2", group = "Autonomous")
public class AutoV2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        double startX = 14;
        double startY = -63;
        double topY = 55+startY;
        double firstX = 31+startX;
        double secondX = 42+startX;
        double wallX = 47+startX;

        //set the starting position
        Pose2d initialPose = new Pose2d(startX, startY, Math.toRadians(-90));

        //initialize our roadrunner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize claw and lift from our mechanisms file
        //Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);
        //Mechanisms.Lift lift = new Mechanisms.Lift(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
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
                ;

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                )
        );
    }
}