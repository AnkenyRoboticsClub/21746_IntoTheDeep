package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

//Call the teleop so it shows up on the driver station
@TeleOp(name = "TeleOpV2", group = "TeleOp")
public class TeleOpV2 extends LinearOpMode {
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
    public void runOpMode() {
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

        //wait for the driver station to start
            waitForStart();

    //primary while loop to call your various functions during driver control from
        while(opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

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

            double rightTrig2 = driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double leftTrig2 = driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            double rightJoy2 = driver2.getRightY();
            double leftJoy2 = driver2.getLeftY();
            arm.armPositionFudgeFactor = (int) (arm.FUDGE_FACTOR * (rightTrig2-leftTrig2+(leftJoy2*2)));
            slide.armPositionFudgeFactor = (int) (slide.FUDGE_FACTOR*rightJoy2);

                if (driver1.getButton(GamepadKeys.Button.START)){
                    lazyImu.get().resetYaw();
                }

            if (driver2.getButton(GamepadKeys.Button.START)){
                slide.slideReset = slide.armMotor.getCurrentPosition();
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

            runningActions.add(new ParallelAction(
                    arm.armRun()
                    ,slide.armRun()
            ));

            /*call our mecanum drive function from ftclib using field centric control,
            if you want robotcentric, change "Field" to "Robot" and remove the imuValue variable,
            if you want exponential drive turned off, change the last variable to false*/
            double xMult = 0.5;
            double yMult = 0.5;
            double rMult = 0.5;
            if (driver1.getButton(GamepadKeys.Button.Y)){
                xMult = 1;
                yMult = 1;
                rMult = 1;
            }
                drive.driveFieldCentric(
                        -driver1.getLeftX()*xMult,
                        -driver1.getLeftY()*yMult,
                        -driver1.getRightX()*rMult,
                        lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                    );
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
