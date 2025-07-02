package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.Servo

class RGBLight(val servo: Servo) {
    private val colorStart = 0.281
    private val colorEnd = 0.722
    private val colorRange = colorEnd - colorStart

    var color: Color = Color.NONE
        set(value) {
            servo.position = value.gbLightPos
            lastColor = color
            field = value
        }

    var lastColor: Color = Color.NONE

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