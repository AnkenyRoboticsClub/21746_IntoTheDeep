package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

//Call the teleop so it shows up on the driver station
@TeleOp(name = "RemakeV1", group = "TeleOp")
public class NewTeleOpV1 extends LinearOpMode {
    private Limelight3A limelight;
    //can have variables and define hardware objects here, anything from here to "waitForStart();" will run in initialization.

    //Change to AdafruitBNO055IMU, BNO055IMU or BHI260IMU based on what you have
    private AdafruitBNO055IMU imu;

    //variables for the automatic imu reset
    double prevImuValue = 0;
    double imuValue = 0;
    double imuDifference = 0;
    double imuWrap = 0;
    private List<Action> runningActions = new ArrayList<>();

    public LazyImu lazyImu;

    boolean armToggleDefault = true;
    boolean armToggle = armToggleDefault;
    int lastArmButtonPressed = 0;

    public static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params PARAMS = new org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params();

    //start of opmode, inside this function will be your main while loop and initialize all hardware objects
    @Override
    public void runOpMode() throws InterruptedException {
        //New LazyIMU
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        //initialize motors, you will need to change these parameters to match your motor setup and names.
        Motor leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        Motor rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        Motor leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        Motor rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        //change the braking behavior, this is mostly personal preference but I recommend leaving this unchanged.
        //leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //reverse motors
        //leftFront.setInverted(true);
        //leftBack.setInverted(true);
        //rightFront.setInverted(true);
        //rightBack.setInverted(true);

        //initialize our mecanum drive from ftclib
        MecanumDrive drive = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );




        //initialize controllers
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        //initialize new mechanisms!
        Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);
        Mechanisms.Wrist wrist = new Mechanisms.Wrist(hardwareMap);
        Mechanisms.Arm arm = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Slide slide = new Mechanisms.Slide(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        //wait for the driver station to start
        waitForStart();

        double xin = 0;
        double yin = 0;

        //primary while loop to call your various functions during driver control from
        while(opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            /*LLResult result = limelight.getLatestResult();
            // First, tell Limelight which way your robot is facing
            //double robotYaw = imu.getAngularOrientation().firstAngle;
            double robotYaw = lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    RobotLog.ii("DbgLog", "MT2 Location:", "(" + x + ", " + y + ")");
                }
            }*/
            /*LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("Field X ",botpose.getPosition().x*23);
                }
            }*/


            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            /*if (lastArmButtonPressed!=4){
                armToggle = armToggleDefault;
            }*/

            //read controller buttons
            driver1.readButtons();
            driver2.readButtons();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    //telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                    xin = botpose.getPosition().x*39.37;
                    yin = botpose.getPosition().y*39.37;
                    telemetry.addData("MT1 Location Inches", "(xin: " + (int)xin + ", yin: " + (int)yin + ")");
                    RobotLog.ii("DbgLog", "MT1 Location Inches"+ "(xin: " + (int)xin + ", yin: " + (int)yin + ")");
                    telemetry.addData("MT1 Heading and IMU Heading:", "MT1: " + (int) botpose.getOrientation().getYaw(AngleUnit.DEGREES)  + ", IMU: " + (int) lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                    //if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                    // left right msec
                    //gamepad1.rumble(0.9, 0, 200);

                    //red
                    double xdiff1 = Math.abs(xin-(-43));
                    double ydiff1 = Math.abs(yin-(-43));
                    //blue
                    double xdiff2 = Math.abs(xin-(43));
                    double ydiff2 = Math.abs(yin-(43));

                    double dirDiff = Math.abs(225-lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                    double maxDiff = 4;
                    double maxDirDiff = 7;

                    RobotLog.ii("DbgLog", "redx"+xdiff1+" redy"+ydiff1+" bluex"+xdiff2+" bluey"+ydiff2);

                    if((xdiff1<maxDiff&&ydiff1<maxDiff)||(xdiff2<maxDiff&&ydiff2<maxDiff)){
                        double leftPower = 0;
                        if(xdiff1<maxDiff&&ydiff1<maxDiff){
                            leftPower = 1-((xdiff1+ydiff1)/(maxDiff+maxDiff));
                        } else {
                            leftPower = 1-((xdiff2+ydiff2)/(maxDiff+maxDiff));
                        }
                        double rightPower = 0;
                        if (dirDiff<maxDirDiff){
                            rightPower = 1-(dirDiff/maxDirDiff);
                        }
                        telemetry.addData("Vibration ", "Left:" + leftPower + " Right: " + rightPower);
                        gamepad1.rumble(leftPower, rightPower, 100);
                    } else {
                        telemetry.addData("Vibration ", "Left:" + 0 + " Right: " + 0);
                        gamepad1.rumble(0, 0, 100);
                    }


                        /*leftFront.setInverted(true);
                        leftBack.setInverted(true);
                        Pose2d initialPose = new Pose2d(xin, yin, botpose.getOrientation().getYaw(AngleUnit.RADIANS));
                        org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive drive2 = new org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive(hardwareMap, initialPose);
                        TrajectoryActionBuilder score = drive2.actionBuilder(initialPose)
                                .strafeToLinearHeading(new Vector2d(-49.75, -49.75), Math.toRadians(225))
                                ;
                        runningActions.add(new ParallelAction(
                                score.build()
                        ));*/
                    //}

                }
            }

            double rightTrig2 = driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double leftTrig2 = driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightJoy2 = driver2.getRightY();
            double leftJoy2 = driver2.getLeftY();

            if (driver1.getButton(GamepadKeys.Button.START)){
                lazyImu.get().resetYaw();
            }

            if (driver2.wasJustPressed(GamepadKeys.Button.START)){
                slide.slideReset = slide.armMotor.getCurrentPosition()-slide.target;//-slide.target;
            }

            if (driver2.getButton(GamepadKeys.Button.A)) {
                lastArmButtonPressed = 1;
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
                lastArmButtonPressed = 2;
                runningActions.add(new ParallelAction(
                        arm.armClear()
                        ,slide.armClear()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.Y)) {
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                lastArmButtonPressed = 3;
                runningActions.add(new ParallelAction(
                        arm.armCollapse(),
                        intake.intakeOff(),
                        wrist.foldInWrist()
                        ,slide.armCollapse()
                ));
            }  else if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                lastArmButtonPressed = 4;
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist(),
                        arm.armScoreHigh(),
                        slide.armScoreHigh()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                runningActions.add(new ParallelAction(
                        wrist.foldOutWrist(),
                        arm.armScoreLow(),
                        slide.armScoreLow()
                ));
            } else if (driver2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                runningActions.add(new ParallelAction(
                        arm.armCollectLow(),
                        intake.intakeCollect(),
                        wrist.foldOutWrist()
                        ,slide.armCollapse()
                ));
            }  else if (driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                lastArmButtonPressed = 5;
                runningActions.add(new ParallelAction(
                        arm.armAttachHangingHook(),
                        intake.intakeOff(),
                        wrist.foldInWrist()
                        ,slide.armAttachHangingHook()
                ));
            }  else if (driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                lastArmButtonPressed = 6;
                runningActions.add(new ParallelAction(
                        arm.armScoreSpecimen(),
                        wrist.foldOutWristSpecimen()
                        ,slide.armScoreSpecimen()
                ));
            }

            if (Math.abs(rightTrig2-leftTrig2+(leftJoy2*2))>0.2) {
                arm.armPositionFudgeFactor = (int) (arm.FUDGE_FACTOR * (rightTrig2 - leftTrig2 + (leftJoy2 * 2)));
                runningActions.add(new ParallelAction(
                        arm.armRun()
                ));
            } else if (arm.armPositionFudgeFactor!=0){
                arm.armPositionFudgeFactor = 0;
                runningActions.add(new ParallelAction(
                        arm.armRun()
                ));
            }

            if (Math.abs(rightJoy2)>0.4){
                slide.armPositionFudgeFactor = (int) (slide.FUDGE_FACTOR*rightJoy2);
                runningActions.add(new ParallelAction(
                        slide.armRun()
                ));
            } else if (slide.armPositionFudgeFactor!=0){
                slide.armPositionFudgeFactor = 0;
                runningActions.add(new ParallelAction(
                        slide.armRun()
                ));
            }
            //note: need to make in go back to zero when you release!!!

            /*call our mecanum drive function from ftclib using field centric control,
            if you want robotcentric, change "Field" to "Robot" and remove the imuValue variable,
            if you want exponential drive turned off, change the last variable to false*/
            double xMult = 1;
            double yMult = 1;
            double rMult = 1;
            if (driver1.getButton(GamepadKeys.Button.Y)){
                xMult = 0.5;
                yMult = 0.5;
                rMult = 0.5;
            }
            /*if (!driver1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                leftFront.setInverted(false);
                leftBack.setInverted(false);
                rightFront.setInverted(false);
                rightBack.setInverted(false);*/
            drive.driveFieldCentric(
                    -driver1.getLeftX() * xMult,
                    -driver1.getLeftY() * yMult,
                    -driver1.getRightX() * rMult,
                    lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
            );
            //}
            telemetry.addData("lazyImu: ", lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("arm encoder: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("arm target: ", arm.armMotor.getTargetPosition());
            telemetry.addData("slide encoder: ", slide.armMotor.getCurrentPosition());
            telemetry.addData("slide target: ", slide.armMotor.getTargetPosition());
            telemetry.addData("slide reset: ", slide.slideReset);
            telemetry.update();
        }
        limelight.stop();
    }
}
