package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

const val cachingThreshold = 0.005
open class CachingDcMotorSimple(val motor: DcMotorSimple) {
    private var lastPower = 0.0
    open fun setPower(power: Double) {
        if ((abs(power - lastPower) > cachingThreshold) || (power == 0.0 && lastPower != power)) {
            motor.power = power
            lastPower = power
        }
    }
}

class CachingDcMotor(motor: DcMotor) : CachingDcMotorSimple(motor), DcMotor by motor {
    override fun setPower(power: Double) {
        super<CachingDcMotorSimple>.setPower(power)
    }
}

class CachingDcMotorEx(motor: DcMotorEx) : CachingDcMotorSimple(motor), DcMotorEx by motor {
    override fun setPower(power: Double) {
        super<CachingDcMotorSimple>.setPower(power)
    }

    val currentCache = Caching(motor::isOverCurrent, 100.0)
    override fun isOverCurrent() = currentCache.get()
}
class CachingCRServo(servo: CRServo) : CachingDcMotorSimple(servo), CRServo by servo {
    override fun setPower(power: Double) {
        super<CachingDcMotorSimple>.setPower(power)
    }
}

class CachingVoltageSensor(val voltageSensor: VoltageSensor) : VoltageSensor by voltageSensor {
    val cache = Caching(voltageSensor::getVoltage)
    override fun getVoltage(): Double {
        return cache.get()
    }
}

open class Caching<T>(val getter: () -> T, val timeoutMS: Double = 500.0) {
    private val timeSinceUpdate = ElapsedTime()
    private var last = getter.invoke()
    fun get(): T {
        if (timeSinceUpdate.milliseconds() > timeoutMS) {
            last = getter.invoke();
            timeSinceUpdate.reset();
        }
        return last;
    }
}
