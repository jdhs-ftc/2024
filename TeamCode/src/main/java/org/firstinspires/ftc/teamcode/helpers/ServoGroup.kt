package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

/**
 * Link a group of motors
 * All WRITES will go to all motors
 * All READS will go only to motor 1
 * EXCEPT: isOverCurrent and isBusy will be an OR of all motors
 * setDirection will be IGNORED, MUST USE setDirections
 */
class ServoGroup(vararg val servos: Servo) : Servo {
    // i shouldnt have made this vararg there's no reason to
    constructor(
        servo1: Servo,
        servo2: Servo,
        direction1: Servo.Direction,
        direction2: Servo.Direction
    ) : this(servo1, servo2) {
        setDirections(direction1, direction2)
    }

    fun setDirections(vararg directions: Servo.Direction) {
        assert(directions.size == servos.size)
        directions.forEachIndexed { index, direction -> servos[index].direction = direction }
    }

    fun scaleRange(servo: Int, min: Double, max: Double) = servos[servo].scaleRange(min, max)


    override fun getManufacturer(): HardwareDevice.Manufacturer = servos[0].manufacturer

    override fun getDeviceName(): String = servos[0].deviceName

    override fun getConnectionInfo(): String = servos[0].connectionInfo

    override fun getVersion(): Int = servos[0].version

    override fun resetDeviceConfigurationForOpMode() =
        servos.forEach { it.resetDeviceConfigurationForOpMode() }

    override fun close() = servos.forEach { it.close() }
    override fun getController(): ServoController = servos[0].controller

    override fun getPortNumber(): Int = servos[0].portNumber

    override fun setDirection(direction: Servo.Direction?) {
        // DO NOTHING
    }

    override fun getDirection(): Servo.Direction = servos[0].direction


    override fun setPosition(position: Double) {
        servos.forEach { it.position = position }
    }

    override fun getPosition(): Double = servos[0].position


    override fun scaleRange(min: Double, max: Double) {
        servos.forEach { it.scaleRange(min, max) }
    }

}