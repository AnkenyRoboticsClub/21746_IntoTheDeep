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

import com.qualcomm.robotcore.util.RobotLog;

public class Mechanisms {

    //class to create a wrist
    public static class Wrist {
        private Servo wrist;
        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }
        public class FoldInWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                RobotLog.ii("DbgLog", "Run: Wrist Pos: 1");
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
                RobotLog.ii("DbgLog", "Run: Wrist Pos: 0.38");
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
                RobotLog.ii("DbgLog", "Run: Wrist Pos: 0.7");
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
        private final int intakeTime =50000;//50000;
        private final int depositTime =50000;//50000;
        //how many times it runs so that it will let it run for a bit before moving to the next action in auto

        private long starting = System.currentTimeMillis();
        private long currentTime=0;
        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(CRServo.class, "intake");
        }

        //implement action class in our intake collect function.

        public class IntakeCollect implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    timer = 0;
                    intake.setPower(1);
                    initialized = true;
                    /*RobotLog.ii("DbgLog", "Init: Intake Collect");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);*/
                    starting = System.currentTimeMillis();
                    currentTime = 0;
                } else {
                    currentTime = System.currentTimeMillis()-starting;
                    timer++;
                }
                /*if (!initialized){
                    intake.setPower(1);
                    firstTime = false;
                    initialized = true;
                    RobotLog.ii("DbgLog", "Init: Intake Collect");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                }
                //return false;
                if (!firstTime) {
                    //timer
                    firstTime = true;
                    timer = 0;
                } else {
                    timer++;

                }*/
                //if (timer>intakeTime) {
                if (currentTime>1500){
                    //firstTime = false;
                    /*RobotLog.ii("DbgLog", "End: Intake Collect");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                    RobotLog.ii("DbgLog", "Time: "+ (currentTime));*/
                    timer = 0;
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
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    timer = 0;
                    intake.setPower(0);
                    initialized = true;
                    RobotLog.ii("DbgLog", "Init: Intake Off");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                }
                timer = 0;
                RobotLog.ii("DbgLog", "End: Intake Off");
                RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                return false;

            }
        }
        //allow the function to be able to be called from other files
        public Action intakeOff() {
            return new Intake.IntakeOff();
        }

        public class IntakeDeposit implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    timer = 0;
                    intake.setPower(-0.75);
                    initialized = true;
                    /*RobotLog.ii("DbgLog", "Init: Intake Deposit");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);*/
                    starting = System.currentTimeMillis();
                    currentTime = 0;
                } else {
                    currentTime = System.currentTimeMillis()-starting;
                    timer++;
                }
                /*if (!initialized){
                    intake.setPower(-0.5);
                    firstTime = false;
                    initialized = true;
                    RobotLog.ii("DbgLog", "Init: Intake Deposit");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                }
                //return false;
                if (!firstTime) {
                    //timer
                    firstTime = true;
                    timer = 0;
                } else {
                    timer++;
                    //RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                }*/
                //if (timer>depositTime) {
                if (currentTime>900){
                    //firstTime = false;
                    /*RobotLog.ii("DbgLog", "End: Intake Deposit");
                    RobotLog.ii("DbgLog", "Intake Timer: "+timer);
                    RobotLog.ii("DbgLog", "Time: "+ (currentTime));*/
                    timer = 0;
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

    public static class Arm {
        final double ARM_TICKS_PER_DEGREE =
                28 // number of encoder ticks per rotation of the bare motor
                        * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                        * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                        * 1/360.0; // we want ticks per degree, not per rotation
        //values copied from old TeleOp
        final int ARM_COLLAPSED_INTO_ROBOT  = 10;
        final int ARM_COLLECT               = (int) (17 * ARM_TICKS_PER_DEGREE);
        final int ARM_COLLECT_LOW           = (int) (0 * ARM_TICKS_PER_DEGREE);
        final int ARM_CLEAR_BARRIER         = (int) (25 * ARM_TICKS_PER_DEGREE);
        final int ARM_SCORE_SPECIMEN        = (int) (64 * ARM_TICKS_PER_DEGREE);
        final int ARM_SCORE_SAMPLE_IN_LOW   = (int) (83 * ARM_TICKS_PER_DEGREE);
        final int ARM_SCORE_SAMPLE_IN_HIGH   = (int) (90 * ARM_TICKS_PER_DEGREE);//87
        final int ARM_ATTACH_HANGING_HOOK   = (int) (110 * ARM_TICKS_PER_DEGREE);
        final int ARM_HANG   = (int) (135 * ARM_TICKS_PER_DEGREE);
        final int ARM_WINCH_ROBOT           = (int) (10  * ARM_TICKS_PER_DEGREE);
        final int FUDGE_FACTOR = (int) (10 * ARM_TICKS_PER_DEGREE);
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
            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        public boolean RunToPos(@NonNull TelemetryPacket packet) {
            ((DcMotorEx) armMotor).setTargetPositionTolerance(50);
            armMotor.setTargetPosition(target+armPositionFudgeFactor);
            int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+1;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int armDistance = Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition());
            /*if (armDistance < 1200) {
                ((DcMotorEx) armMotor).setVelocity(1500);
            } else {
                ((DcMotorEx) armMotor).setVelocity(2100);
            }*/
            if (armDistance>tolerance) {
                // true causes the action to rerun
                return true;
            } else {
                //false stops action rerun and stops the arm
                RobotLog.ii("DbgLog", "Arm Finished: Target: "+armMotor.getTargetPosition()+" Position: "+armMotor.getCurrentPosition()+" Velocity: "+((DcMotorEx) armMotor).getVelocity());
                return false;
            }
        }
        public class ArmScoreLow implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SAMPLE_IN_LOW;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreLow() {
            return new ArmScoreLow();
        }
        public class ArmScoreHigh implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SAMPLE_IN_HIGH;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreHigh() {
            return new ArmScoreHigh();
        }
        public class ArmCollapse implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_COLLAPSED_INTO_ROBOT;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armCollapse(){
            return new ArmCollapse();
        }
        public class ArmCollect implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_COLLECT;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armCollect(){
            return new ArmCollect();
        }

        public class ArmCollectLow implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_COLLECT_LOW;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armCollectLow(){
            return new ArmCollectLow();
        }
        public class ArmHangingHook implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_ATTACH_HANGING_HOOK;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armAttachHangingHook(){
            return new ArmHangingHook();
        }

        public class ArmHang implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_HANG;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armHang()
        {
            return new ArmHang();
        }
        public class ArmClear implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_CLEAR_BARRIER;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armClear(){
            return new ArmClear();
        }
        public class ArmScoreSpecimen implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SPECIMEN;
                    armMotor.setPower(0.8);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreSpecimen(){
            return new ArmScoreSpecimen();
        }
        public class ArmRun implements Action {
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
                return RunToPos(packet);
            }
        }
        public Action armRun(){
            return new ArmRun();
        }
    }

    public static class Slide {
        final int ARM_COLLAPSED_INTO_ROBOT  = 0;
        final int ARM_COLLECT               = -2000;
        final int ARM_CLEAR_BARRIER         = -1000;
        final int ARM_SCORE_SPECIMEN        = -400;
        final int ARM_SCORE_SAMPLE_IN_LOW   = -10;
        final int ARM_SCORE_SAMPLE_IN_HIGH   = -2800;//-2500
        final int ARM_RELEASE_SAMPLE_IN_HIGH   = -2700;
        final int ARM_ATTACH_HANGING_HOOK   = 0;
        final int FUDGE_FACTOR = 300;
        //public Motor arm;
        public DcMotor armMotor;
        public int target;
        public int armPositionFudgeFactor;
        public int slideReset = 0;
        private int stuckCounter = 0;
        private int lastPos = 0;

        public int power = 1;
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
            armMotor = hardwareMap.get(DcMotor.class, "arm_slide"); //the arm motor
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        public boolean RunToPos(@NonNull TelemetryPacket packet) {
            ((DcMotorEx) armMotor).setTargetPositionTolerance(50);//100
            armMotor.setTargetPosition(target+armPositionFudgeFactor+slideReset);
            int tolerance = ((DcMotorEx) armMotor).getTargetPositionTolerance()+2;
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int armDistance = Math.abs(armMotor.getCurrentPosition()-armMotor.getTargetPosition());
            /*if (armDistance < 1200) {
                ((DcMotorEx) armMotor).setVelocity(1500);
            } else {
                ((DcMotorEx) armMotor).setVelocity(2100);
            }*/
            if(lastPos==armMotor.getCurrentPosition()){
                stuckCounter++;
            } else {
                stuckCounter = 0;
            }
            if ((armDistance>tolerance)&&stuckCounter<100) {
                // true causes the action to rerun
                RobotLog.ii("DbgLog", "Slide Running: Target: "+armMotor.getTargetPosition()+" Position: "+armMotor.getCurrentPosition()+" Stuck Counter: "+stuckCounter);
                lastPos = armMotor.getCurrentPosition();
                return true;
            } else {
                //false stops action rerun and stops the arm
                if(stuckCounter>=100){
                    stuckCounter = 0;
                    armMotor.setTargetPosition(lastPos);
                    RobotLog.ii("DbgLog", "Slide Canceled: Target: "+armMotor.getTargetPosition()+" Position: "+armMotor.getCurrentPosition()+" Stuck Counter: "+stuckCounter);
                } else {
                    RobotLog.ii("DbgLog", "Slide Finished: Target: " + armMotor.getTargetPosition() + " Position: " + armMotor.getCurrentPosition());
                }
                return false;
            }
        }
        public class ArmScoreLow implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SAMPLE_IN_LOW;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreLow() {
            return new Slide.ArmScoreLow();
        }
        public class ArmScoreHigh implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SAMPLE_IN_HIGH;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreHigh() {
            return new Slide.ArmScoreHigh();
        }

        public class ArmReleaseHigh implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_RELEASE_SAMPLE_IN_HIGH;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armReleaseHigh() {
            return new Slide.ArmReleaseHigh();
        }
        public class ArmCollapse implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_COLLAPSED_INTO_ROBOT;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armCollapse(){
            return new Slide.ArmCollapse();
        }
        public class ArmCollect implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_COLLECT;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armCollect(){
            return new Slide.ArmCollect();
        }
        public class ArmHangingHook implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_ATTACH_HANGING_HOOK;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armAttachHangingHook(){
            return new Slide.ArmHangingHook();
        }
        public class ArmClear implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_CLEAR_BARRIER;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armClear(){
            return new Slide.ArmClear();
        }
        public class ArmScoreSpecimen implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    target = ARM_SCORE_SPECIMEN;
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armScoreSpecimen(){
            return new Slide.ArmScoreSpecimen();
        }
        public class ArmRun implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.setPower(power);
                    initialized = true;
                }
                return RunToPos(packet);
            }
        }
        public Action armRun(){
            return new Slide.ArmRun();
        }
    }
}
