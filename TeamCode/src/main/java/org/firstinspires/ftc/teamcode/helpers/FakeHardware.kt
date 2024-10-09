package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

class FakeServo : Servo {
    var direction: Servo.Direction = Servo.Direction.FORWARD
    var position: Double = 0.0
    override fun getController(): ServoController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setDirection(direction: Servo.Direction?) {
        this.direction = direction!!
    }

    override fun getDirection(): Servo.Direction? {
        return direction
    }
    override fun setPosition(position: Double) {
        this.position = position
    }

    override fun getPosition(): Double {
        return position
    }

    override fun scaleRange(min: Double, max: Double) {
        // do nothing
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String? {
        return "Fake Servo"
    }

    override fun getConnectionInfo(): String? {
        return "Fake Servo"
    }

    override fun getVersion(): Int {
        return 12087
    }

    override fun resetDeviceConfigurationForOpMode() {
        // do nothing
    }

    override fun close() {
        // do nothing
    }

}