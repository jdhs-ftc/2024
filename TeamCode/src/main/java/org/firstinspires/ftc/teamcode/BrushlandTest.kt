package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.motor.MotorControl
import org.firstinspires.ftc.teamcode.motor.MotorControl.BLColor

@TeleOp
class BrushlandTest: OpMode() {
    lateinit var digital0: DigitalChannel
    lateinit var digital1: DigitalChannel
    lateinit var color: MotorControl.BLColor
    lateinit var light: RGBLight
    override fun init() {
        digital0 = hardwareMap.digitalChannel["digital0"]
        digital1 = hardwareMap.digitalChannel["digital1"]
        color = BLColor(digital0,digital1)
        light = RGBLight(hardwareMap.servo.get("rgb"))
    }

    override fun loop() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("digital0",digital0.state)
        telemetry.addData("digital1",digital1.state)
        telemetry.addData("color",color.color)
        telemetry.update()

        when (color.color) {
            BLColor.Color.NONE -> light.color = RGBLight.Color.OFF
            BLColor.Color.RED -> light.color = RGBLight.Color.RED
            BLColor.Color.BLUE -> light.color = RGBLight.Color.BLUE
            BLColor.Color.YELLOW -> light.color = RGBLight.Color.YELLOW
        }
    }
}