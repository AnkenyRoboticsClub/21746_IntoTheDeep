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
        //init stuff from mechanism class
        Mechanisms.Arm arm = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);
        Mechanisms.Wrist wrist = new Mechanisms.Wrist(hardwareMap);
        Mechanisms.Slide slide = new Mechanisms.Slide(hardwareMap);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-49.5, -49.5), Math.toRadians(225))
                .afterTime(0, arm.armScoreHigh())
                .afterTime(0, wrist.foldOutWrist())
                ;

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                .afterTime(0, slide.armCollapse())
                .strafeToLinearHeading(new Vector2d(-40, -30), Math.toRadians(160))
                .afterTime(0, arm.armClear())
                ;

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-39, -31), Math.toRadians(160))
                .afterTime(0, intake.intakeCollect())
                ;
        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-49.5, -49.5), Math.toRadians(225))
                .afterTime(0, arm.armScoreHigh())
                ;

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47, -47), Math.toRadians(225))
                .afterTime(0, slide.armCollapse())
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
                        , intake.intakeDeposit()
                        , traj2.build()
                        , arm.armCollectLow()
                        , traj3.build()
                        , intake.intakeCollect()
                        , intake.intakeOff()
                        , traj4.build()
                        , slide.armScoreHigh()
                        , intake.intakeDeposit()
                        , traj5.build()
                )
        );
    }
}