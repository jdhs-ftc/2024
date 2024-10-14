package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController

class FakeServo : Servo {
    internal var fakeDirection: Servo.Direction = Servo.Direction.FORWARD
    internal var fakePosition: Double = 0.0
    override fun getController(): ServoController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setDirection(direction: Servo.Direction?) {
        this.fakeDirection = direction!!
    }

    override fun getDirection(): Servo.Direction? {
        return fakeDirection
    }
    override fun setPosition(position: Double) {
        this.fakePosition = position
    }

    override fun getPosition(): Double {
        return fakePosition
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