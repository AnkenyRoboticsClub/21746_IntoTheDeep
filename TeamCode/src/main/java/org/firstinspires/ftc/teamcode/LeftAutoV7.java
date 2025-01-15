package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "LeftV6", group = "Autonomous")
public class LeftAutoV7 extends LinearOpMode {

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

        arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(initialPose)
                .afterTime(0.7, arm.armScoreHigh())
                .afterTime(0.7, wrist.foldOutWrist())
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                ;

        TrajectoryActionBuilder getBlock1 = scorePreload.endTrajectory().fresh()
                .afterTime(0.5, slide.armCollapse())
                .strafeToLinearHeading(new Vector2d(-33, -40), Math.toRadians(160))
                .afterTime(0, arm.armCollectLow())
                .afterTime(0, intake.intakeOff())
                .strafeToLinearHeading(new Vector2d(-34, -32), Math.toRadians(160))
                .afterTime(0, intake.intakeCollect())
                .strafeToLinearHeading(new Vector2d(-42, -31), Math.toRadians(160))
                ;

        TrajectoryActionBuilder scoreBlock1 = getBlock1.endTrajectory().fresh()
                .afterTime(0, arm.armScoreHigh())
                .afterTime(1, slide.armScoreHigh())
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                ;

        TrajectoryActionBuilder getBlock2 = scoreBlock1.endTrajectory().fresh()
                .afterTime(0.5, slide.armCollapse())
                .afterTime(1, arm.armCollectLow())
                .afterTime(0, intake.intakeOff())
                .strafeToLinearHeading(new Vector2d(-44, -25), Math.toRadians(180))
                .afterTime(0, intake.intakeCollect())
                .strafeToLinearHeading(new Vector2d(-52, -25), Math.toRadians(180))
                ;

        TrajectoryActionBuilder scoreBlock2 = getBlock2.endTrajectory().fresh()
                .afterTime(0, arm.armScoreHigh())
                .afterTime(1, slide.armScoreHigh())
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                ;

        TrajectoryActionBuilder getBlock3 = scoreBlock2.endTrajectory().fresh()
                .afterTime(0.5, slide.armCollapse())
                .afterTime(1, arm.armCollectLow())
                .afterTime(0, intake.intakeOff())
                .strafeToLinearHeading(new Vector2d(-54, -25), Math.toRadians(180))
                .afterTime(0, intake.intakeCollect())
                .strafeToLinearHeading(new Vector2d(-62, -25), Math.toRadians(180))
                ;

        TrajectoryActionBuilder scoreBlock3 = getBlock3.endTrajectory().fresh()
                .afterTime(0, arm.armScoreHigh())
                .afterTime(1, slide.armScoreHigh())
                .strafeToLinearHeading(new Vector2d(-50.25, -50.25), Math.toRadians(225))
                ;

        TrajectoryActionBuilder park = scoreBlock3.endTrajectory().fresh()
                .afterTime(0.5, slide.armCollapse())
                .strafeToLinearHeading(new Vector2d(-40, -20), Math.toRadians(225))
                .afterTime(0, intake.intakeOff())
                .afterTime(0.1, arm.armAttachHangingHook())
                .splineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(225)),Math.toRadians(0))
                .afterTime(0, wrist.foldInWrist())
                .strafeToLinearHeading(new Vector2d(-25, -11), Math.toRadians(180))
                ;


        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        Actions.runBlocking(
                new SequentialAction(
                        //score preload
                        scorePreload.build(),
                        slide.armScoreHigh(),
                        intake.intakeDeposit(),
                        //get 1st
                        getBlock1.build(),
                        //score 1st
                        scoreBlock1.build(),
                        intake.intakeDeposit(),
                        //get 2nd
                        getBlock2.build(),
                        //score 2nd
                        scoreBlock2.build(),
                        intake.intakeDeposit(),
                        //get 3rd
                        getBlock3.build(),
                        //score 3rd
                        scoreBlock3.build(),
                        intake.intakeDeposit(),
                        //park
                        park.build(),
                        arm.armHang()
                )
        );
    }
}