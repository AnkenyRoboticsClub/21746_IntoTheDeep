package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import java.util.Timer;

public class Mechanisms {

    //class to create a wrist
    public static class Wrist {
        private Servo wrist;
        //create the claw object from hardware map

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        //implement action class in our close claw function.

        public class FoldInWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                wrist.setPosition(1);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action foldInWrist() {
            return new Wrist.FoldInWrist();
        }
        //create an foldinwrist function by implementing action class


        public class FoldOutWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                wrist.setPosition(0.38);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action foldOutWrist() {
            return new Wrist.FoldOutWrist();
        }

        public class FoldOutWristSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                wrist.setPosition(0.7);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action foldOutWristSpecimen() {
            return new Wrist.FoldOutWristSpecimen();
        }
    }

    public static class Intake {
        private CRServo intake;
        private boolean firstTime = false;
        private double timer = 0;
        private int intakeTime =100000;
        //create the claw object from hardware map

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
        }

        //implement action class in our intake collect function.

        public class IntakeCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                intake.setPower(1);
                //return false;
                if (!firstTime) {
                    //timer
                    firstTime = true;
                    timer = 0;
                } else {
                    timer++;
                }
                if (timer>intakeTime) {
                    firstTime = false;
                    return false;

                } else {
                    return true;
                }
            }
        }
        //allow the function to be able to called from other files
        public Action intakeCollect() {
            return new Intake.IntakeCollect();
        }
        //create an intakecollect function by implementing action class

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                intake.setPower(0);
                //return false;
                if (!firstTime) {
                    //timer
                    firstTime = true;
                    timer = 0;
                } else {
                    timer++;
                }
                if (timer>intakeTime) {
                    firstTime = false;
                    return false;

                } else {
                    return true;
                }
            }
        }
        //allow the function to be able to be called from other files
        public Action intakeOff() {
            return new Intake.IntakeOff();
        }

        public class IntakeDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                intake.setPower(-0.5);
                //return false;
                if (!firstTime) {
                    //timer
                    firstTime = true;
                    timer = 0;
                } else {
                    timer++;
                }
                if (timer>intakeTime) {
                    firstTime = false;
                    return false;

                } else {
                    return true;
                }
            }
        }
        //allow the function to be able to be called from other files
        public Action intakeDeposit() {
            return new Intake.IntakeDeposit();
        }
    }

    //use as a example for servos!
    //class to create a claw
    /*public static class Claw {
        private Servo claw;
        //create the claw object from hardware map

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        //implement action class in our close claw function.

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                claw.setPosition(0.55);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action closeClaw() {
            return new Claw.CloseClaw();
        }
        //create an openclaw function by implementing action class

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                claw.setPosition(1.0);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }
    */

    public static class Arm {
        final double ARM_TICKS_PER_DEGREE =
                28 // number of encoder ticks per rotation of the bare motor
                        * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                        * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                        * 1/360.0; // we want ticks per degree, not per rotation
        //values copied from TeleOpV3
        final double ARM_COLLAPSED_INTO_ROBOT  = 0;
        final double ARM_COLLECT               = 17 * ARM_TICKS_PER_DEGREE;
        final double ARM_CLEAR_BARRIER         = 25 * ARM_TICKS_PER_DEGREE;
        final double ARM_SCORE_SPECIMEN        = 68 * ARM_TICKS_PER_DEGREE;
        final double ARM_SCORE_SAMPLE_IN_LOW   = 83 * ARM_TICKS_PER_DEGREE;
        final double ARM_SCORE_SAMPLE_IN_HIGH   = 84 * ARM_TICKS_PER_DEGREE;
        final double ARM_ATTACH_HANGING_HOOK   = 100 * ARM_TICKS_PER_DEGREE;
        final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;
        final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
        //public Motor arm;
        public DcMotor armMotor;
        public int target;
        public int armPositionFudgeFactor;
        //create lift from hardwaremap and initialize it

        public Arm(HardwareMap hardwareMap) {
            /*//initialize our lift from hardwareMap
            arm = new Motor(hardwareMap, "left_arm", Motor.GoBILDA.RPM_117);
            //set the braking mode to brake when theres no power given so it better holds target position
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            arm.setRunMode(Motor.RunMode.PositionControl);
            //set the lift motor direction
            //arm.setInverted(true);
            //set position coefficient of the lift, (p value)
            arm.setPositionCoefficient(0.001);*/
            armMotor = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        public class ArmRunPosition implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armMotor.setTargetPosition(target+armPositionFudgeFactor);
                //((DcMotorEx) armMotor).setVelocity(2100);
                int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+2;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //return false;
                if ((Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition())>tolerance)) {
                    // true causes the action to rerun
                    return true;
                } else {
                    //false stops action rerun and stops the arm
                    //arm.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public class ArmRunPositionCollect implements Action {
            //made to fix my cheat coding b4
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armMotor.setTargetPosition((int) ARM_COLLECT);
                //((DcMotorEx) armMotor).setVelocity(2100);
                int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+2;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //return false;
                if ((Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition())>tolerance)) {
                    // true causes the action to rerun
                    return true;
                } else {
                    //false stops action rerun and stops the arm
                    //arm.set(0);
                    return false;
                }
            }
        }

        public class ArmRunPositionScoreLow implements Action {
            //made to fix my cheat coding b4
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armMotor.setTargetPosition((int) ARM_SCORE_SAMPLE_IN_LOW+10);
                //((DcMotorEx) armMotor).setVelocity(2100);
                int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+2;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //return false;
                if ((Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition())>tolerance)) {
                    // true causes the action to rerun
                    return true;
                } else {
                    //false stops action rerun and stops the arm
                    //arm.set(0);
                    return false;
                }
            }
        }

        public class ArmRunPositionCollapse implements Action {
            //made to fix my cheat coding b4
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
                //((DcMotorEx) armMotor).setVelocity(2100);
                int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+2;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //return false;
                if ((Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition())>tolerance)) {
                    // true causes the action to rerun
                    return true;
                } else {
                    //false stops action rerun and stops the arm
                    //arm.set(0);
                    return false;
                }
            }
        }

        public Action armScoreLow() {
            target = (int) ARM_SCORE_SAMPLE_IN_LOW;
            return new ArmRunPosition();
        }

        public Action armScoreLowFix() {
            return new ArmRunPositionScoreLow();
        }
        public Action armCollapse(){
            target = (int) ARM_COLLAPSED_INTO_ROBOT;
            return new ArmRunPosition();
        }

        public Action armCollapseFix(){
            return new ArmRunPositionCollapse();
        }

        public Action armCollect(){
            target = (int) ARM_COLLECT;
            return new ArmRunPosition();
        }

        public Action armCollectFix(){
            return new ArmRunPositionCollect();
        }

        public Action armAttachHangingHook(){
            target = (int) ARM_ATTACH_HANGING_HOOK;
            return new ArmRunPosition();
        }
        public Action armClear(){
            target = (int) ARM_CLEAR_BARRIER;
            return new ArmRunPosition();
        }

        public Action armScoreSpecimen(){
            target = (int) ARM_SCORE_SPECIMEN;
            return new ArmRunPosition();
        }

        public Action armRun(){
            return new ArmRunPosition();
        }
    }

    public static class Slide {
        final double ARM_COLLAPSED_INTO_ROBOT  = 0;
        final double ARM_COLLECT               = -2000;
        final double ARM_CLEAR_BARRIER         = -1000;
        final double ARM_SCORE_SPECIMEN        = 400;
        final double ARM_SCORE_SAMPLE_IN_LOW   = 10;
        final double ARM_SCORE_SAMPLE_IN_HIGH   = -2500;
        final double ARM_ATTACH_HANGING_HOOK   = 0;
        final double FUDGE_FACTOR = 15;
        //public Motor arm;
        public DcMotor armSlideMotor;
        public int target;
        public int armPositionFudgeFactor;
        //create lift from hardwaremap and initialize it

        public Slide(HardwareMap hardwareMap) {
            /*//initialize our lift from hardwareMap
            arm = new Motor(hardwareMap, "left_arm", Motor.GoBILDA.RPM_117);
            //set the braking mode to brake when theres no power given so it better holds target position
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            arm.setRunMode(Motor.RunMode.PositionControl);
            //set the lift motor direction
            //arm.setInverted(true);
            //set position coefficient of the lift, (p value)
            arm.setPositionCoefficient(0.001);*/
            armSlideMotor = hardwareMap.get(DcMotor.class, "arm_slide"); //the arm motor
            armSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSlideMotor.setTargetPosition(0);
            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        public class ArmRunPosition implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armSlideMotor.setPower(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armSlideMotor.setTargetPosition(target+armPositionFudgeFactor);
                //((DcMotorEx) armMotor).setVelocity(2100);
                int tolerance = ((DcMotorEx) armSlideMotor).getTargetPositionTolerance()+2;
                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //return false;
                if ((Math.abs(armSlideMotor.getCurrentPosition()-armSlideMotor.getTargetPosition())>tolerance)) {
                    // true causes the action to rerun
                    return true;
                } else {
                    //false stops action rerun and stops the arm
                    //arm.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }

        public Action armScoreLow() {
            target = (int) ARM_SCORE_SAMPLE_IN_LOW;
            return new ArmRunPosition();
        }
        public Action armCollapse(){
            target = (int) ARM_COLLAPSED_INTO_ROBOT;
            return new ArmRunPosition();
        }

        public Action armCollect(){
            target = (int) ARM_COLLECT;
            return new ArmRunPosition();
        }

        public Action armAttachHangingHook(){
            target = (int) ARM_ATTACH_HANGING_HOOK;
            return new ArmRunPosition();
        }
        public Action armClear(){
            target = (int) ARM_CLEAR_BARRIER;
            return new ArmRunPosition();
        }

        public Action armScoreSpecimen(){
            target = (int) ARM_SCORE_SPECIMEN;
            return new ArmRunPosition();
        }

        public Action armRun(){
            return new ArmRunPosition();
        }
    }

    /* //can use as an example for ftc lib PID but default one is better (less bugs)
    //lift class (this will require an encoder plugged into the motor)
    public static class Lift {
        private Motor lift;
        //create lift from hardwaremap and initialize it

        public Lift(HardwareMap hardwareMap) {
            //initialize our lift from hardwareMap
            lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_30);
            //set the braking mode to brake when theres no power given so it better holds target position
            lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            lift.setRunMode(Motor.RunMode.PositionControl);
            //set the lift motor direction
            lift.setInverted(true);
            //set position coefficient of the lift, (p value)
            lift.setPositionCoefficient(0.001);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.set(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                lift.setTargetPosition(3000);
                if (!lift.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    lift.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the lifts target position to down position
                lift.setTargetPosition(10);
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.set(-0.8);
                    initialized = true;
                }

                //if the lift isn't at the target position then repeat the loop
                if (!lift.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    lift.set(0);
                    return false;
                }
                // overall, the action powers the lift down until it goes below
                // 100 encoder ticks, then powers it off
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }*/
    /*public static class Intake {
        private Motor intake;
        //create the claw object from hardware map

        public Intake(HardwareMap hardwareMap) {
            //initialize our intake from hardwareMap
            intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
            //set the braking mode to float when theres no power given so it doesn't do anything
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            //set the runmode to raw power
            intake.setRunMode(Motor.RunMode.RawPower);
            //set the direction of the motor
            intake.setInverted(false);
        }

        //implement action class in our spin intake forward function.

        public class spinForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when intake spinforward is run, spin the intake forward
                intake.set(0.8);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action spinForward() {
            return new Intake.spinForward();
        }
        //create an spin backward function by implementing action class

        public class spinBackward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when spin backward is run, spin the intake backwards
                intake.set(-0.8);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action spinBackward() {
            return new Intake.spinBackward();
        }
    }*/
}
