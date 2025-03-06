package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TestTeleOpV1", group = "TeleOp")
public class NewTeleOpV1 extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    public LazyImu lazyImu;
    private Limelight3A limelight;
    boolean driverControlled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        double xin = 0;
        double yin = 0;

        // Assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        //Imu
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));

        //Drive Motors
        DcMotorEx leftFront, leftBack, rightBack, rightFront;
        leftFront = drive.leftFront;
        leftBack = drive.leftBack;
        rightBack = drive.rightBack;
        rightFront = drive.rightFront;

        //Gamepads
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        //Mechanisms
        Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);
        Mechanisms.Wrist wrist = new Mechanisms.Wrist(hardwareMap);
        Mechanisms.Arm arm = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Slide slide = new Mechanisms.Slide(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Make sure to call drive.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
            drive.updatePoseEstimate();

            // Retrieve your pose
            Pose2d myPose = drive.pose;

            telemetry.addData("Roadrunner X:", myPose.position.x);
            telemetry.addData("Roadrunner Y:", myPose.position.y);
            telemetry.addData("Roadrunner Heading:", myPose.heading);
            RobotLog.ii("DbgLog", "Roadrunner Pose" + "(x: " + (int) myPose.position.x + ", y: " + (int) myPose.position.y + ", heading: " + myPose.heading + ")");

            //Camera MT1
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    xin = x * 39.37;
                    yin = y * 39.37;
                    telemetry.addData("MT1 Location Inches", "(xin: " + (int) xin + ", yin: " + (int) yin + ")");
                    RobotLog.ii("DbgLog", "MT1 Location Inches" + "(xin: " + (int) xin + ", yin: " + (int) yin + ")");
                    telemetry.addData("Heading:", "MT1: " + (int) botpose.getOrientation().getYaw(AngleUnit.DEGREES) + ", IMU: " + (int) lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    //fix roadrunner pose
                    if (driver1.wasJustPressed(GamepadKeys.Button.X)) {
                        double poseX = 0;
                        double poseY = 0;
                        if(yin>0) {
                            poseX = -xin;
                            poseY = -yin;
                        } else {
                            poseX = xin;
                            poseY = yin;
                        }
                        double poseOrientation = lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+Math.toRadians(90);
                        RobotLog.ii("DbgLog", "Pose Update Conversion" + "(x: " + (int) poseX + ", y: " + (int) poseY + ")");
                        drive.pose = new Pose2d(poseX, poseY, poseOrientation);
                    }
                }
            }

            driverControlled = !driver1.getButton(GamepadKeys.Button.Y);
            if (driver1.wasJustPressed(GamepadKeys.Button.Y)){
                TrajectoryActionBuilder score = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(-49.75, -49.75), Math.toRadians(225))
                        ;
                runningActions.add(new ParallelAction(
                        score.build()
                ));
            }

            driver1.readButtons();
            driver2.readButtons();

            if(driverControlled) {
                double xMult = 1;
                double yMult = 1;
                double rMult = 1;
                if (driver1.getButton(GamepadKeys.Button.Y)) {
                    xMult = 0.5;
                    yMult = 0.5;
                    rMult = 0.5;
                }

                double y = -gamepad1.left_stick_y * xMult; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x * yMult;
                double rx = gamepad1.right_stick_x * rMult;

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (driver1.getButton(GamepadKeys.Button.START)) {
                    lazyImu.get().resetYaw();
                }

                double botHeading = lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);
            }

            //Actions from TeleOp V4
            TelemetryPacket packet = new TelemetryPacket();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            if (driver2.getButton(GamepadKeys.Button.A)) {
                runningActions.add(new ParallelAction(
                        arm.armCollect(),
                        wrist.foldOutWrist(),
                        intake.intakeCollect()
                        , slide.armCollect()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                runningActions.add(new ParallelAction(
                        intake.intakeCollect()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.B)) {
                runningActions.add(new ParallelAction(
                        intake.intakeOff()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                runningActions.add(new ParallelAction(
                        intake.intakeDeposit()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.X)) {
                runningActions.add(new ParallelAction(
                        arm.armClear()
                        , slide.armClear()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.Y)) {
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                runningActions.add(new ParallelAction(
                        arm.armCollapse(),
                        intake.intakeOff(),
                        wrist.foldInWrist()
                        , slide.armCollapse()
                ));
            } else if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist(),
                        arm.armScoreHigh(),
                        slide.armScoreHigh()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist(),
                        arm.armScoreLow(),
                        slide.armScoreLow()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                runningActions.add(new ParallelAction(
                        arm.armCollectLow(),
                        intake.intakeCollect(),
                        wrist.foldOutWrist()
                        , slide.armCollapse()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                runningActions.add(new ParallelAction(
                        arm.armAttachHangingHook(),
                        intake.intakeOff(),
                        wrist.foldInWrist()
                        , slide.armAttachHangingHook()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.add(new ParallelAction(
                        arm.armScoreSpecimen(),
                        wrist.foldOutWristSpecimen()
                        , slide.armScoreSpecimen()
                ));
            }

            //Manual Arm Adjustments
            double rightTrig2 = driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double leftTrig2 = driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightJoy2 = driver2.getRightY();
            double leftJoy2 = driver2.getLeftY();

            if (driver2.wasJustPressed(GamepadKeys.Button.START)) {
                slide.slideReset = slide.armMotor.getCurrentPosition() - slide.target;//-slide.target;
            }

            if (Math.abs(rightTrig2 - leftTrig2 + (leftJoy2 * 2)) > 0.2) {
                arm.armPositionFudgeFactor = (int) (arm.FUDGE_FACTOR * (rightTrig2 - leftTrig2 + (leftJoy2 * 2)));
                runningActions.add(new ParallelAction(
                        arm.armRun()
                ));
            } else if (arm.armPositionFudgeFactor != 0) {
                arm.armPositionFudgeFactor = 0;
                runningActions.add(new ParallelAction(
                        arm.armRun()
                ));
            }

            if (Math.abs(rightJoy2) > 0.4) {
                slide.armPositionFudgeFactor = (int) (slide.FUDGE_FACTOR * rightJoy2);
                runningActions.add(new ParallelAction(
                        slide.armRun()
                ));
            } else if (slide.armPositionFudgeFactor != 0) {
                slide.armPositionFudgeFactor = 0;
                runningActions.add(new ParallelAction(
                        slide.armRun()
                ));
            }

            //More telemetery
            telemetry.addData("lazyImu: ", lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("arm encoder: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("arm target: ", arm.armMotor.getTargetPosition());
            telemetry.addData("slide encoder: ", slide.armMotor.getCurrentPosition());
            telemetry.addData("slide target: ", slide.armMotor.getTargetPosition());
            telemetry.addData("slide reset: ", slide.slideReset);
            telemetry.update();
        }
    }
}
