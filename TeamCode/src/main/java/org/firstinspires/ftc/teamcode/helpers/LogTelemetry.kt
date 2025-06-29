package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry

class LogTelemetry @JvmOverloads constructor(val base: String = "Telemetry/"): Telemetry {
    val lastData = mutableMapOf<String, Any>()
    val lastTime = mutableMapOf<String, Double>()
    // unsure about this default
    // also is there a better way to implement this?
    private var msTransmissionIntervalField = 10

    override fun update(): Boolean {
        FlightRecorder.write("${base}TELEMETRY_UPDATE", object {
            // match the format of other messages; parsable by AdvantageScope
            @JvmField
            val timestamp = System.nanoTime()
        })
        return true
    }

    override fun addLine(): Telemetry.Line? {
        return null
    }

    override fun addLine(lineCaption: String): Telemetry.Line? {
        addData(lineCaption, true)
        return null
    }

    override fun addData(
        caption: String,
        format: String,
        vararg args: Any
    ): Telemetry.Item? {
        return this.addData(caption, String.format(format, *args))
    }

    override fun addData(
        caption: String,
        value: Any
    ): Telemetry.Item? {
        if (lastData[caption] != value // don't spam logs with same value
            && lastTime[caption]?.let {
                now() * 1000 - it > msTransmissionInterval
            } == true
        ) {
            try {
                FlightRecorder.write("$base$caption", value)
            } catch (e: Exception) { // can't log empty classes, among other issues; don't crash
                RobotLog.ww("RRLogTelemetry", "Failed to log $base$caption: $value because $e")
                FlightRecorder.write("$base${caption}_FAILED", "Failed to log $value because $e")
            }
            lastData[caption] = value
            lastTime[caption] = now() * 1000
        }
        return null
    }

    override fun getMsTransmissionInterval(): Int {
        return msTransmissionIntervalField
    }

    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
        msTransmissionIntervalField = msTransmissionInterval
    }

    // all below ignored

    override fun isAutoClear(): Boolean {
        return false
    }

    override fun <T : Any> addData(
        caption: String,
        valueProducer: Func<T?>
    ): Telemetry.Item? {
        return null
    }

    override fun <T : Any> addData(
        caption: String,
        format: String,
        valueProducer: Func<T?>
    ): Telemetry.Item? {
        return null
    }

    override fun removeItem(item: Telemetry.Item): Boolean {
        return false
    }

    override fun clear() {
        // do nothing
    }

    override fun clearAll() {
        // do nothing
    }

    override fun addAction(action: Runnable): Any? {
        // do nothing
        return null
    }

    override fun removeAction(token: Any): Boolean {
        return false
    }

    override fun speak(text: String) {
        // do nothing
    }

    override fun speak(
        text: String,
        languageCode: String,
        countryCode: String
    ) {
        // do nothing
    }


    override fun removeLine(line: Telemetry.Line): Boolean {
        return false
    }

    override fun setAutoClear(autoClear: Boolean) {
        // do nothing
    }

    override fun getItemSeparator(): String? {
        return "|"
    }

    override fun setItemSeparator(itemSeparator: String?) {
        // do nothing
    }

    override fun getCaptionValueSeparator(): String {
        return " : "
    }

    override fun setCaptionValueSeparator(captionValueSeparator: String) {
        // do  nothing
    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat) {
        // do nothing
    }

    override fun log(): Telemetry.Log? {
        return null
    }
}