package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.Servo

class RGBLight(val servo: Servo) {
    val colorStart = 0.281
    val colorEnd = 0.722
    val colorRange = colorEnd - colorStart

    enum class Color(val pos: Double) {
        OFF(0.0),
        RED(0.281),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0)
    }

    fun setColor(color: Color) {
        servo.position = color.pos
    }
    fun setColor(hue: Double) {
        servo.position = hueToPos(hue)
    }

    /** Hue is 0-255 **/
    fun hueToPos(hue: Double): Double {
        if (hue < 0) return colorStart
        if (hue > 255) return colorEnd
        return colorStart + (hue / 255) * colorRange
    }
    // doesn't really work
    fun updateRainbow() {
        servo.position = ((servo.position + 0.0005) - colorStart) % colorRange + colorStart
    }
}