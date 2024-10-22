package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
}
class CachingCRServo(servo: CRServo) : CachingDcMotorSimple(servo), CRServo by servo {
    override fun setPower(power: Double) {
        super<CachingDcMotorSimple>.setPower(power)
    }
}



