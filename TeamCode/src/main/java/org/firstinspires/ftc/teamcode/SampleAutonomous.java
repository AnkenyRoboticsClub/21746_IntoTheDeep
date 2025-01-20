package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "SAMPLE_AUTONOMOUS", group = "Autonomous")
@Disabled
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
                .splineToLinearHeading(new Pose2d(-30, 22, Math.toRadians(0)), Math.toRadians(0))
                //move lift up after 1.5 seconds
                /*.afterTime(1.5, lift.liftUp())*/;

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                //simple movement, spline to a linear heading, so it will go to it's original position
                //(0,0) with a heading of 0 degrees
                .splineToLinearHeading(new Pose2d(-50, 23, Math.toRadians(0)), Math.toRadians(0))
                //move lift down after 5 inches traveled
                /*.afterDisp(5.0, lift.liftDown())*/;

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-50, 32, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(0)), Math.toRadians(0));
        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-50, 32, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-45, 42, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-2, 42, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-40, 47, Math.toRadians(0)), Math.toRadians(0));

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        traj2.build(),
                        traj3.build(),
                        traj4.build(),
                        traj5.build(),
                        traj6.build(),
                        traj7.build(),
                        traj8.build()
                )
        );
    }
}