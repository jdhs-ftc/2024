package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.helpers.RGBLight

@TeleOp
class BrushlandAnalogTest: OpMode() {
    lateinit var analog: AnalogInput
    lateinit var light: RGBLight
    override fun init() {
        analog = hardwareMap.analogInput["brushland"]
        light = RGBLight(hardwareMap.servo.get("rgb"))
    }

    override fun loop() {
        val voltage = analog.voltage
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("rawAnalog", voltage)
        telemetry.addData("hue", voltage / 3.3 * 360)
        telemetry.update()

        light.setHue(voltage / 3.3 * 255)
    }
}