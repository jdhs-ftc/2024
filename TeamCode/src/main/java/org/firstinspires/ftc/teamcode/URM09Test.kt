package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.helpers.URM09

@TeleOp
class URM09Test: OpMode() {
    lateinit var urm09: URM09
    lateinit var light: RGBLight
    override fun init() {
        urm09 = URM09(hardwareMap.analogInput.get("depositArmEncoder"))
        light = RGBLight(hardwareMap.servo.get("rgb"))
    }

    override fun loop() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addData("voltage", urm09.channel.voltage)
        telemetry.addData("distanceCm", urm09.distanceCm)
        telemetry.addData("distanceIn", urm09.distanceIn)
        telemetry.update()
    }
}