package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.Servo

class RGBLight(val servo: Servo) {
    private val colorStart = 0.281
    private val colorEnd = 0.722
    private val colorRange = colorEnd - colorStart

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

    var color: Color = Color.OFF
        set(value) {
            servo.position = value.pos
            field = value
        }

    fun setHue(hue: Double) {
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