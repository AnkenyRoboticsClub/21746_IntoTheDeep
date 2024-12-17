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
@Autonomous(name = "LeftV3", group = "Autonomous")
public class LeftAutoV3 extends LinearOpMode {

    @Override
    public void runOpMode() {

        /*DcMotorEx leftFront, leftBack, rightBack, rightFront;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        //set the starting position
        Pose2d initialPose = new Pose2d(-33, -63, Math.toRadians(-90));

        //initialize our roadrunner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize claw and lift from our mechanisms file
        //Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);
        //Mechanisms.Lift lift = new Mechanisms.Lift(hardwareMap);
        Mechanisms.Arm arm = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);
        Mechanisms.Wrist wrist = new Mechanisms.Wrist(hardwareMap);

        //moves to score preloaded sample
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                .afterTime(0, arm.armScoreLowFix())
                .afterTime(0, wrist.foldOutWrist())
                ;

        //get into position to pick up 1st sample
        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25, -36), Math.toRadians(-20))
                //.afterTime(0.1, arm.armCollect())
                //.afterTime(0, wrist.foldOutWrist())
                //.afterTime(0, intake.intakeCollect())
                ;

        //moves to ACTUALLY pick up the sample
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-32, -32), Math.toRadians(-20))
                ;

        //moves to corner to score sample
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                //.afterTime(0, intake.intakeOff())
                //.afterTime(0, arm.armScoreLow())
                ;

        //moves to position to pick 2nd sample
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-36, -25), Math.toRadians(0))
                .afterTime(0, wrist.foldInWrist())
                .afterTime(0, arm.armCollapseFix())
                ;

        //moves to ACTUALLY pick 2nd sample
        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-43, -25), Math.toRadians(0))
                ;

        //moves to original position to deposit
        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                ;

        //position to pick up 3rd sample
        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-49, -25), Math.toRadians(0))
                ;

        //ACTUALLY pick sample 3
        TrajectoryActionBuilder traj9 = traj8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-56, -25), Math.toRadians(0))
                ;

        //moves to deposit
        TrajectoryActionBuilder traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45))
                ;

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                        , wrist.foldOutWrist()
                        , intake.intakeDeposit()
                        , traj2.build()
                        , arm.armCollectFix()
                        , wrist.foldOutWrist()
                        , intake.intakeCollect()
                        , traj3.build()
                        , intake.intakeCollect()
                        , intake.intakeOff()
                        , arm.armScoreLowFix()
                        , traj4.build()
                        , intake.intakeDeposit()
                        , traj5.build()
                        , arm.armCollectFix()
                        , wrist.foldOutWrist()
                        , intake.intakeCollect()
                        , traj6.build()
                        , intake.intakeCollect()
                        , intake.intakeOff()
                        , traj7.build()
                        , intake.intakeDeposit()
                        , traj8.build()
                        , arm.armCollectFix()
                        , wrist.foldOutWrist()
                        , intake.intakeCollect()
                        , traj9.build()
                        , intake.intakeCollect()
                        , intake.intakeOff()
                        , traj10.build()
                        , intake.intakeDeposit()
                )
        );
    }
}