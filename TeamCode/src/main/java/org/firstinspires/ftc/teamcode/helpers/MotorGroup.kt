package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

/**
 * Link a group of motors
 * All WRITES will go to all motors
 * All READS will go only to motor 1
 * EXCEPT: isOverCurrent and isBusy will be an OR of all motors
 * setDirection will be IGNORED, MUST USE setDirections
 */
class MotorGroup(vararg val motors: DcMotorEx) : DcMotorEx {
    // i shouldnt have made this vararg there's no reason to
    constructor(motor1: DcMotorEx, motor2: DcMotorEx, direction1: DcMotorSimple.Direction, direction2: DcMotorSimple.Direction) : this(motor1, motor2) {
        setDirections(direction1, direction2)
    }

    fun setDirections(vararg directions: DcMotorSimple.Direction) {
        assert(directions.size == motors.size)
        directions.forEachIndexed { index, direction -> motors[index].direction = direction }
    }

    override fun setMotorEnable() = motors.forEach { it.setMotorEnable() }


    override fun setMotorDisable() = motors.forEach { it.setMotorDisable() }

    override fun isMotorEnabled(): Boolean = motors.all { it.isMotorEnabled }


    override fun setVelocity(p0: Double) = motors.forEach { it.velocity = p0 }


    override fun setVelocity(
        p0: Double,
        p1: AngleUnit?
    ) = motors.forEach { it.setVelocity(p0, p1) }


    override fun getVelocity() = motors[0].velocity


    override fun getVelocity(p0: AngleUnit?) = motors[0].velocity

    @Deprecated("Deprecated in Java")
    override fun setPIDCoefficients(
        p0: DcMotor.RunMode?,
        p1: PIDCoefficients?
    ) = motors.forEach { it.setPIDCoefficients(p0, p1) }


    override fun setPIDFCoefficients(
        p0: DcMotor.RunMode?,
        p1: PIDFCoefficients?
    ) = motors.forEach { it.setPIDFCoefficients(p0, p1) }


    override fun setVelocityPIDFCoefficients(
        p0: Double,
        p1: Double,
        p2: Double,
        p3: Double
    ) = motors.forEach { it.setVelocityPIDFCoefficients(p0, p1, p2, p3) }


    override fun setPositionPIDFCoefficients(p0: Double) =
        motors.forEach { it.setPositionPIDFCoefficients(p0) }


    @Deprecated("Deprecated in Java")
    override fun getPIDCoefficients(p0: DcMotor.RunMode?): PIDCoefficients? =
        motors[0].getPIDCoefficients(p0)


    override fun getPIDFCoefficients(p0: DcMotor.RunMode?): PIDFCoefficients? =
        motors[0].getPIDFCoefficients(p0)


    override fun setTargetPositionTolerance(p0: Int) =
        motors.forEach { it.targetPositionTolerance = p0 }


    override fun getTargetPositionTolerance(): Int = motors[0].targetPositionTolerance


    override fun getCurrent(p0: CurrentUnit?): Double = motors[0].getCurrent(p0)


    override fun getCurrentAlert(p0: CurrentUnit?): Double = motors[0].getCurrentAlert(p0)


    override fun setCurrentAlert(
        p0: Double,
        p1: CurrentUnit?
    ) = motors.forEach { it.setCurrentAlert(p0, p1) }


    override fun isOverCurrent() = motors.any { it.isOverCurrent }


    override fun getMotorType(): MotorConfigurationType = motors[0].motorType

    override fun setMotorType(p0: MotorConfigurationType) = motors.forEach { it.motorType = p0 }


    override fun getController(): DcMotorController = motors[0].controller


    override fun getPortNumber() = motors[0].portNumber


    override fun setZeroPowerBehavior(p0: DcMotor.ZeroPowerBehavior?) = motors.forEach {
        it.zeroPowerBehavior =
            p0
    }


    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior = motors[0].zeroPowerBehavior


    @Deprecated("Deprecated in Java")
    override fun setPowerFloat() = motors.forEach { it.setPowerFloat() }


    override fun getPowerFloat(): Boolean = motors[0].powerFloat


    override fun setTargetPosition(p0: Int) = motors.forEach { it.targetPosition = p0 }

    override fun getTargetPosition(): Int = motors[0].targetPosition

    override fun isBusy(): Boolean = motors.any { it.isBusy }

    override fun getCurrentPosition(): Int = motors[0].currentPosition

    override fun setMode(p0: DcMotor.RunMode) = motors.forEach { it.mode = p0 }

    override fun getMode(): DcMotor.RunMode = motors[0].mode

    override fun setDirection(p0: DcMotorSimple.Direction) {} // INTENTIONALLY IGNORE, USE SETDIRECTIONS
    override fun getDirection(): DcMotorSimple.Direction = motors[0].direction

    override fun setPower(p0: Double) = motors.forEach { it.power = p0 }

    override fun getPower(): Double = motors[0].power

    override fun getManufacturer(): HardwareDevice.Manufacturer = motors[0].manufacturer

    override fun getDeviceName(): String = motors[0].deviceName

    override fun getConnectionInfo(): String = motors[0].connectionInfo

    override fun getVersion(): Int = motors[0].version

    override fun resetDeviceConfigurationForOpMode() =
        motors.forEach { it.resetDeviceConfigurationForOpMode() }

    override fun close() = motors.forEach { it.close() }

}