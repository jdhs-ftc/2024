package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

public class MotorActions {
    public final MotorControl motorControl;
    public final Extendo extendo;
    public final Deposit deposit;

    public MotorActions(MotorControl motorControl) {
        this.motorControl = motorControl;
        this.extendo = new Extendo();
        this.deposit = new Deposit();

    }
    public Action waitUntilFinished() {
        return t -> motorControl.closeEnough();
    }

    public Action update() {
        return t -> {
            motorControl.update();
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }


    public Action log(String message) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket t) {
                System.out.println(message);
                return false;
            }
        };
    }


    public class Extendo {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.extendo.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.extendo.closeEnough();
                }
            };
        }

        public Action moveUp() {
            return setTargetPosition(1200);
        }
        public Action moveDown() {
            return setTargetPosition(40);
        }
    }
    public class Deposit {
        public Action setTargetPosition(double position) {
            return t -> {
                motorControl.extendo.setTargetPosition(position);
                return false;
            };
        }
        public Action waitUntilFinished() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket t) {
                    return !motorControl.extendo.closeEnough();
                }
            };
        }

        public Action moveUp() {
            return setTargetPosition(1200);
        }
        public Action moveDown() {
            return setTargetPosition(40);
        }
    }

}
