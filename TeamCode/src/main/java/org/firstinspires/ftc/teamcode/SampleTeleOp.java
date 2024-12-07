package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

//Call the teleop so it shows up on the driver station
@TeleOp(name = "SAMPLE_TELEOP", group = "TeleOp")
public class SampleTeleOp extends LinearOpMode {
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

    public static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params PARAMS = new org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.Params();

    //start of opmode, inside this function will be your main while loop and initialize all hardware objects
    @Override
    public void runOpMode() {
        //New LazyIMU
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        //IMU intialization, you will need to change this to  match your imu
            /*imu = hardwareMap.get(AdafruitBNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);*/
        // Retrieve the IMU from the hardware map
        /*IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);*/

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
            com.arcrobotics.ftclib.drivebase.MecanumDrive drive = new MecanumDrive(
                    leftFront,
                    rightFront,
                    leftBack,
                    rightBack
            );

        //initialize controllers
            GamepadEx driver1 = new GamepadEx(gamepad1);
            GamepadEx driver2 = new GamepadEx(gamepad2);


        //initialize claw servo object from our mechanisms file
            //Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);

        //initialize lift motor object from our mechanimsms file
            //Mechanisms.Lift lift = new Mechanisms.Lift(hardwareMap);

        //intialize intake motor from our mechanisms file
            //Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);

        //initialize new mechanisms!
            Mechanisms.Intake intake = new Mechanisms.Intake(hardwareMap);
            Mechanisms.Wrist wrist = new Mechanisms.Wrist(hardwareMap);
            Mechanisms.Arm arm = new Mechanisms.Arm(hardwareMap);


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


            //read controller buttons
                driver1.readButtons();
                driver2.readButtons();

                //armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger) + gamepad1.right_trigger + (-gamepad1.left_trigger));
            double rightTrig2 = driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double leftTrig2 = driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            arm.armPositionFudgeFactor = (int) (arm.FUDGE_FACTOR * (rightTrig2-leftTrig2));

                if (driver1.getButton(GamepadKeys.Button.START)){
                    lazyImu.get().resetYaw();
                }

                if (driver2.getButton(GamepadKeys.Button.A)) {
                    /*armPosition = ARM_COLLECT;
                    //liftPosition = LIFT_COLLAPSED;
                    wrist.setPosition(WRIST_FOLDED_OUT);
                    intake.setPower(INTAKE_COLLECT);*/
                    runningActions.add(new ParallelAction(
                            arm.armCollect(),
                            wrist.foldOutWrist(),
                            intake.intakeCollect()
                    ));
                } else if (driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    runningActions.add(new ParallelAction(
                            intake.intakeDeposit()
                    ));
                } else if (driver2.getButton(GamepadKeys.Button.B)) {
                    runningActions.add(new ParallelAction(
                            intake.intakeOff()
                    ));
                } else if (driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    runningActions.add(new ParallelAction(
                            intake.intakeCollect()
                    ));
                } else if (driver2.getButton(GamepadKeys.Button.X)) {
                    runningActions.add(new ParallelAction(
                            arm.armClear()
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
                    ));
                }  else if (driver2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.add(new ParallelAction(
                            arm.armScoreLow(),
                            wrist.foldOutWrist()
                    ));
                }  else if (driver2.getButton(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.add(new ParallelAction(
                            arm.armAttachHangingHook(),
                            intake.intakeOff(),
                            wrist.foldInWrist()
                    ));
                }  else if (driver2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    runningActions.add(new ParallelAction(
                            arm.armScoreSpecimen(),
                            wrist.foldInWrist()
                    ));
                }

            runningActions.add(new ParallelAction(
                    arm.armRun()
            ));

            //example of claw control
                /*if (driver1.wasJustPressed(GamepadKeys.Button.A)) {
                    runningActions.add(new SequentialAction(
                            //claw.openClaw()
                            intake.intakeCollect()

                    ));
                }
                else if (driver1.wasJustPressed(GamepadKeys.Button.B)) {
                    runningActions.add(new SequentialAction(
                            //claw.closeClaw()
                            intake.intakeOff()
                    ));
                }

            //example of lift control
                //can use .wasJustPressed because the lift target position only needs to change once.
                    if(driver1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        runningActions.add(new SequentialAction(
                                //lift.liftUp()
                                arm.armScoreLow()
                        ));
                    }
                    else if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        runningActions.add(new SequentialAction(
                                //lift.liftDown()
                                arm.armCollapse()
                        ));
                    }

            //example of intake control
                //use an if else statement first so that theres a tolerance before it starts moving the motor
                    if(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                            //call the spin intake forward function
                        runningActions.add(new SequentialAction(
                                //intake.spinForward()
                        ));
                    }
                    else if(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                            //call the spin intake backward function
                        runningActions.add(new SequentialAction(
                                //intake.spinBackward()
                        ));
                    }*/


            /*Code to automatically reset the imu to the last known setting if it gets reset because of static.
            I highly recommend leaving this unchanged unless you know what you're doing*/
            /*
            //get imu value as a variable
                imuValue = imu.getAngularOrientation().firstAngle + imuDifference;
            //find difference between current and previous values, wrapped around 360
                imuWrap = prevImuValue - imuValue;
                imuWrap -= 360 * Math.floor(0.5 + imuWrap / 360);
            //if difference between previous
                if(Math.abs(imuWrap) > 30) {
                    leftFront.set(0); rightFront.set(0); leftBack.set(0); rightBack.set(0);
                    //give driver feedback in the form of a short rumble if the imu has to reset because of static
                    gamepad1.rumble(500);
                    imu.initialize();
                    imuDifference = imuWrap;
                    imuValue = imu.getAngularOrientation().firstAngle + imuDifference;
                }
                prevImuValue = imuValue;

            */
            //Code to manually reset the imu, you can change this to whatever button you want
                /*if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    imu.initialize();
                    imuValue = 0;
                    prevImuValue = 0;
                    imuDifference = 0;
                }*/
            /*call our mecanum drive function from ftclib using field centric control,
            if you want robotcentric, change "Field" to "Robot" and remove the imuValue variable,
            if you want exponential drive turned off, change the last variable to false*/
            double xMult = 0.5;
            double yMult = 0.5;
            if (driver1.getButton(GamepadKeys.Button.Y)){
                xMult = 1;
                yMult = 1;
            }
                drive.driveFieldCentric(
                        -driver1.getLeftX()*xMult,
                        -driver1.getLeftY()*yMult,
                        -driver1.getRightX(),
                        lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                    );
            telemetry.addData("lazyImu: ", lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("arm encoder: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("arm target: ", arm.armMotor.getTargetPosition());
            telemetry.update();
        }
    }
}
