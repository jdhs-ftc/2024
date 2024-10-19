package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import org.firstinspires.ftc.teamcode.helpers.ColorRangefinder.AnalogMode
import org.firstinspires.ftc.teamcode.helpers.ColorRangefinder.DigitalMode
import org.firstinspires.ftc.teamcode.helpers.ColorRangefinder.PinNum
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.pow

/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: [...](https://docs.brushlandlabs.com)
 */
class ColorRangefinder(val base: RevColorSensorV3) {
    private val i2c: I2cDeviceSynchSimple = base.getDeviceClient()

    /**
     * Configure Pin 0 to be in digital mode and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    fun setPin0Digital(digitalMode: DigitalMode, lowerBound: Double, higherBound: Double) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound)
    }

    /**
     * Configure Pin 1 to be in digital mode and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    fun setPin1Digital(digitalMode: DigitalMode, lowerBound: Double, higherBound: Double) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound)
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    fun setPin0DigitalMaxDistance(digitalMode: DigitalMode, mmRequirement: Double) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement)
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    fun setPin1DigitalMaxDistance(digitalMode: DigitalMode, mmRequirement: Double) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement)
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    fun setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200.0)
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    fun setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200.0)
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    fun setPin0Analog(analogMode: AnalogMode, denominator: Int) {
        val denom0 = (denominator and 0xFF).toByte()
        val denom1 = ((denominator and 0xFF00) shr 8).toByte()
        i2c.write(PinNum.PIN0.modeAddress.toInt(), byteArrayOf(analogMode.value, denom0, denom1))
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    fun setPin0Analog(analogMode: AnalogMode) {
        setPin0Analog(analogMode, if (analogMode == AnalogMode.DISTANCE) 100 else 0xFFFF)
    }

    fun getCalibration(): FloatArray {
        val bytes =
            ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0.toInt(), 16)).order(ByteOrder.LITTLE_ENDIAN)
        return floatArrayOf(bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat())
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    fun setLedBrightness(value: Int) {
        i2c.write8(LED_BRIGHTNESS.toInt(), value)
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    fun setI2cAddress(value: Int) {
        i2c.write8(I2C_ADDRESS_REG.toInt(), value shl 1)
    }

    /**
     * Read distance via I2C
     * @return distance in millimeters
     */
    fun readDistance(): Double {
        val bytes =
            ByteBuffer.wrap(i2c.read(PS_DISTANCE_0.toInt(), 4)).order(ByteOrder.LITTLE_ENDIAN)
        return bytes.getFloat().toDouble()
    }

    private fun setDigital(
        pinNum: PinNum,
        digitalMode: DigitalMode,
        lowerBound: Double,
        higherBound: Double
    ) {
        var lo: Int
        var hi: Int
        if (lowerBound == higherBound) {
            lo = lowerBound.toInt()
            hi = higherBound.toInt()
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = Math.round(lowerBound / 255.0 * 65535).toInt()
            hi = Math.round(higherBound / 255.0 * 65535).toInt()
        } else { // distance in mm
            val calib = getCalibration()
            println(calib.contentToString())
            if (lowerBound < .5) hi = 2048
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound)
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound)
        }

        val lo0 = (lo and 0xFF).toByte()
        val lo1 = ((lo and 0xFF00) shr 8).toByte()
        val hi0 = (hi and 0xFF).toByte()
        val hi1 = ((hi and 0xFF00) shr 8).toByte()
        i2c.write(pinNum.modeAddress.toInt(), byteArrayOf(digitalMode.value, lo0, lo1, hi0, hi1))
    }

    private fun root(n: Double, v: Double): Double {
        var `val` = v.pow(1.0 / abs(n))
        if (n < 0) `val` = 1.0 / `val`
        return `val`
    }

    private fun rawFromDistance(a: Float, b: Float, c: Float, x0: Float, mm: Double): Int {
        return (root(b.toDouble(), (mm - c) / a) + x0).toInt()
    }

    private enum class PinNum(modeAddress: Int) {
        PIN0(0x28), PIN1(0x2D);

        val modeAddress: Byte = modeAddress.toByte()
    }

    init {
        this.i2c.enableWriteCoalescing(true)
    }

    enum class DigitalMode(value: Int) {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);

        val value: Byte = value.toByte()
    }

    enum class AnalogMode(value: Int) {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);

        val value: Byte = value.toByte()
    }

    companion object {
        // other writeable registers
        private const val CALIB_A_VAL_0: Byte = 0x32
        private const val PS_DISTANCE_0: Byte = 0x42
        private const val LED_BRIGHTNESS: Byte = 0x46
        private const val I2C_ADDRESS_REG: Byte = 0x47

        fun invertHue(hue360: Int): Int {
            return ((hue360 - 180) % 360)
        }
    }
}