package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadTest extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("share", gamepad1.share);
        telemetry.addData("ps", gamepad1.ps);
        telemetry.addData("touchpad", gamepad1.touchpad);
        telemetry.addData("touchpad_1", gamepad1.touchpad_finger_1);
        telemetry.addData("touchpad_2", gamepad1.touchpad_finger_2);

        telemetry.update();
    }
}