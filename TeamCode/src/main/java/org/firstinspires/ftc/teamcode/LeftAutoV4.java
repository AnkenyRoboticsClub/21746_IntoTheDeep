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
@Autonomous(name = "LeftV4", group = "Autonomous")
public class LeftAutoV4 extends LinearOpMode {

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
        Mechanisms.Slide slide = new Mechanisms.Slide(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                .afterTime(0, arm.armScoreHigh()) //changing arm class
                //.afterTime(0, slide.armScoreHigh())
                .afterTime(0, wrist.foldOutWrist())
                ;

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-25, -36), Math.toRadians(160))
                ;

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-32, -32), Math.toRadians(160))
                ;
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(225))
                ;
        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-36, -32), Math.toRadians(90))
                .afterTime(0, wrist.foldInWrist())
                .afterTime(0, arm.armCollapse())
                ;

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                        , slide.armScoreHigh()
                        , wrist.foldOutWrist()
                        , intake.intakeDeposit()
                        , slide.armCollapse()
                        , traj2.build()
                        , arm.armCollect() //changing arm class
                        , wrist.foldOutWrist()
                        ,slide.armCollect()
                        , intake.intakeCollect()
                        , traj3.build()
                        , intake.intakeCollect()
                        , intake.intakeOff()
                        , arm.armScoreHigh() //changing arm class
                        , traj4.build()
                        , slide.armScoreHigh()
                        , intake.intakeDeposit()
                        , slide.armCollapse()
                        , traj5.build()
                )
        );
    }
}