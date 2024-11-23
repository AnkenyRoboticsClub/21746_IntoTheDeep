package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "SAMPLE_AUTONOMOUS", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        //set the starting position
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        //initialize our roadrunner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize claw and lift from our mechanisms file
        //Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);
        //Mechanisms.Lift lift = new Mechanisms.Lift(hardwareMap);


        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                //simple movement, spline to a linear heading, so it will go to position
                //(10, 10) with a heading of 90 degrees
                .splineToLinearHeading(new Pose2d(10, 10, Math.toRadians(90)), Math.toRadians(0))
                //move lift up after 1.5 seconds
                /*.afterTime(1.5, lift.liftUp())*/;

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                //simple movement, spline to a linear heading, so it will go to it's original position
                //(0,0) with a heading of 0 degrees
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                //move lift down after 5 inches traveled
                /*.afterDisp(5.0, lift.liftDown())*/;

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

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