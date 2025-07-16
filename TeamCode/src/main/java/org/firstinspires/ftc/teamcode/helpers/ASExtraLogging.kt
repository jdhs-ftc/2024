package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.roundToInt

object ASExtraLogging {
    fun start(opMode: OpMode) {
        FlightRecorder.write("/SystemStats/EpochTimeMicros", System.currentTimeMillis() * 1000)
    }

    var lastGamepad1 = Gamepad()
    var lastGamepad2 = Gamepad()

    fun logGamepads(gamepad1: Gamepad, gamepad2: Gamepad) {
        if (gamepad1.toByteArray() != lastGamepad1.toByteArray()) {
            FlightRecorder.write("/DriverStation/Joystick1", GamepadMessage(gamepad1, "gamepad1"))
            lastGamepad1.copy(gamepad1)
        }
        if (gamepad2.toByteArray() != lastGamepad2.toByteArray()) {
            FlightRecorder.write("/DriverStation/Joystick2", GamepadMessage(gamepad2, "gamepad2"))
            lastGamepad2.copy(gamepad2)
        }
    }



    fun loop(opMode: OpMode) {
        logGamepads(opMode.gamepad1, opMode.gamepad2)
    }
}

class GamepadMessage(gamepad: Gamepad, name: String) {
    @JvmField
    val Name = name
    @JvmField
    val Type = gamepad.type.ordinal
    @JvmField
    val Xbox = gamepad.type == Gamepad.Type.XBOX_360
    @JvmField
    val ButtonValues = arrayOf(gamepad.a, gamepad.b, gamepad.x, gamepad.y, gamepad.guide, gamepad.start, gamepad.back, gamepad.left_bumper, gamepad.right_bumper, gamepad.left_stick_button, gamepad.right_stick_button, gamepad.touchpad, gamepad.touchpad_finger_1, gamepad.touchpad_finger_2, gamepad.ps)
    @JvmField
    val ButtonCount = ButtonValues.size
    fun getPovAngle(gamepad: Gamepad): Int {
        var values = intArrayOf()
        if (gamepad.dpad_up) {
            values += 360
        }
        if (gamepad.dpad_right) {
            values += 90
        }
        if (gamepad.dpad_down) {
            values += 180
        }
        if (gamepad.dpad_left) {
            values += 270
        }
        return if (values.isNotEmpty()) {
            (values.average() % 360).roundToInt()
        } else {
            -1
        }
    }
    @JvmField
    val POVs = arrayOf(getPovAngle(gamepad))

    @JvmField
    val AxisValues = doubleArrayOf(
        gamepad.left_stick_x.toDouble(),
        gamepad.left_stick_y.toDouble(),
        gamepad.right_stick_x.toDouble(),
        gamepad.right_stick_y.toDouble(),
        gamepad.right_trigger.toDouble(),
        gamepad.left_trigger.toDouble(),
    )

    @JvmField
    val AxisTypes = arrayOf(0,1,0,1,3,3)
}