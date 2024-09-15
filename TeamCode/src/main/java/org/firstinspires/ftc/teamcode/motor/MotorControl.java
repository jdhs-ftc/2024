package org.firstinspires.ftc.teamcode.motor;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController;

import java.util.ArrayList;

/**
 * This class is used to control the motor systems on the robot.
 */
public class MotorControl {

    public final Servo groundClaw;
    public final Servo depositClaw;
    public final Slide extendo;
    public final Slide deposit;
    public static ArrayList<ControlledMotor> motors = new ArrayList<>();
    //public final ColorSensor color;

    /**
     * This initializes the arm and slide motors and resets the mode to the default.
     * This should be run before any other methods.
     *
     * @param hardwareMap The hardware map to use to get the motors.
     */
    public MotorControl(@NonNull HardwareMap hardwareMap) {
        extendo = new Slide(hardwareMap, // port 0 of exp hub and chub, equiv to left_back I think
                "extendo",
                new PIDFController.PIDCoefficients(3,0,0));
        extendo.setEncoder(hardwareMap.get(DcMotorEx.class,"left_back"));
        deposit = new Slide(hardwareMap, // port 1 of exp hub and chub,
                // equiv to left_front I think could be wrong though
                "deposit",
                new PIDFController.PIDCoefficients(3,0,0));
        deposit.setEncoder(hardwareMap.get(DcMotorEx.class,"right_back"));


        groundClaw = hardwareMap.get(Servo.class, "groundClaw");
        depositClaw = hardwareMap.get(Servo.class, "depositClaw");
        groundClaw.setPosition(0);
        depositClaw.setPosition(0);

        //color = hardwareMap.get(ColorSensor.class, "color");
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    public void update() {
        for (ControlledMotor motor : motors) {
            motor.update();
        }
    }


    public boolean closeEnough() {
        return motors.stream().allMatch(ControlledMotor::closeEnough);
    }

    public boolean isOverCurrent() {
        return motors.stream().anyMatch(ControlledMotor::isOverCurrent);
    }

    /**
     * This class controls the slide motor.
     */
    public static class Slide extends ControlledMotor {
        boolean reversed = false;
        boolean resetting = false;
        PIDFController pid;
        public RawEncoder encoder;
        /**
         * This initializes the slide motor. This should be run before any other methods.
         *
         * @param hardwareMap The hardware map to use to get the motors.
         */
        public Slide(HardwareMap hardwareMap,String motorName,PIDFController.PIDCoefficients pidCoefficients) {
            super();
            pid = new PIDFController(pidCoefficients);
            motor = hardwareMap.get(DcMotorEx.class, motorName);
            encoder = new RawEncoder(motor);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setCurrentAlert(6, CurrentUnit.AMPS);
            if (reversed) {
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
                encoder.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
                encoder.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        public void setEncoder(DcMotorEx motor) {
            encoder = new RawEncoder(motor);
            if (reversed) {
                encoder.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                encoder.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        public void reset() {
            motor.setPower(0);
            targetPosition = 0;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        /**
         * This updates the slide motor to match the current state. This should be run in a loop.
         */
        public void update() {
            pid.targetPosition = targetPosition;
            if (resetting) {
                if (motor.getCurrent(CurrentUnit.AMPS) > 2.5) {
                    reset();
                    resetting = false;
                }
            } else {
                if (!motor.isOverCurrent()) {
                    motor.setPower(pid.update(motor.getCurrentPosition()));
                } else {
                    motor.setPower(0);

                }
            }
        }

        public void findZero() {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(-0.5);
            resetting = true;
        }


        public boolean closeEnough() {
            return Math.abs(motor.getCurrentPosition() - targetPosition) < 20;
        }

    }

    /**
     * This class controls the claw.
     */

    public abstract static class ControlledMotor {
        public DcMotorEx motor;
        double targetPosition;
        public double getTargetPosition() {
            return targetPosition;
        }
        public void setTargetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }
        public abstract void update();
        public abstract void reset();
        public abstract boolean closeEnough();
        public boolean isOverCurrent() {
            return motor.isOverCurrent();
        }

        public ControlledMotor() {
            motors.add(this);
        }
    }
}
