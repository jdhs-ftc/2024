package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.helpers.RGBLight.Color

@TeleOp
class RGBLightOpMode: OpMode() {
    lateinit var light: RGBLight
    override fun init() {
        light = RGBLight(hardwareMap.servo.get("rgb"))
        light.setColor(Color.VIOLET)
    }

    override fun loop() {
        light.updateRainbow()
        telemetry.addLine("HELLO")
        telemetry.update()
    }
}