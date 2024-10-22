package org.firstinspires.ftc.teamcode.motor

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.FakeServo
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController.PIDCoefficients
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt


/**
 * This class is used to control the motor systems on the robot.
 */
class MotorControl(hardwareMap: HardwareMap) {
    init { // empty the motor list, it's static so remains between op modes by default :skull:
        motors = ArrayList()
    }

    @JvmField
    val extendoArm = ExtendoArm(hardwareMap.get(Servo::class.java, "sArm"), 0.7, 0.5) //0.03, 0.2) // dump pos 0.6, set in class

    @JvmField
    val extendoClaw = Claw(hardwareMap.get(Servo::class.java, "extendoClaw"), 0.7, 0.3)

    @JvmField
    val extendo: Slide = Slide(
        CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "extendo")), // port 0, encoder is  left_front
        PIDFController(PIDCoefficients(0.001, 0.0, 0.0))
    )

    @JvmField
    val depositArm = ServoArm(FakeServo(), 0.0, 1.0) // TODO: CHANGE TO THE RIGHT ONE

    @JvmField
    val depositLid = Claw(FakeServo(), 0.0, 1.0) // TODO CHANGE TO NOT FAKE

    @JvmField
    val depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.2, 0.65)

    @JvmField
    val deposit: Slide

    val pin0 = hardwareMap.digitalChannel.get("digital0")
    val pin1 = hardwareMap.digitalChannel.get("digital1")

    //public final ColorSensor color;
    init {
        extendo.encoder = hardwareMap.get(DcMotorEx::class.java, "left_front")
        extendo.reversed = true

        deposit = Slide(
            CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "deposit")),
            // port 1 of exp hub and chub,
            // encoder is left_back
            PIDFController(
                PIDCoefficients(0.005, 0.0, 0.0),
                PIDFController.FeedforwardFun { a, b -> return@FeedforwardFun 0.15 })
        )
        deposit.encoder = hardwareMap.get(DcMotorEx::class.java, "left_back")
        deposit.reversed = true

        motors.forEach { it.findZero() }
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    fun update() {
        motors.forEach { it.update() }
    }


    fun closeEnough(): Boolean {
        return motors.stream().allMatch { it.closeEnough() }
    }

    val isOverCurrent: Boolean
        get() = motors.stream().anyMatch { it.isOverCurrent }

    /**
     * This class controls the slide motor.
     */
    class Slide(motor: DcMotorEx, val pid: PIDFController) :
        ControlledMotor(motor) {
        var reversed = true // EVERYTHING REVERSED, this working is a coincidence
        var resetting = false
        var encoder: DcMotorEx = motor
        var encoderOffset: Double = 0.0

        init {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.setCurrentAlert(6.0, CurrentUnit.AMPS)
            if (reversed) {
                motor.direction = DcMotorSimple.Direction.REVERSE
            } else {
                motor.direction = DcMotorSimple.Direction.FORWARD
            }
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            targetPosition = 20.0
        }

        /**
         * This stops the slide, sets the state to down, sets the target to 0, and resets the encoder.
         */
        override fun reset() {
            motor.power = 0.0
            targetPosition = 0.0
            position = 0.0
        }


        override fun update() {
            pid.targetPosition = targetPosition
            if (resetting) {
                if (motor.getCurrent(CurrentUnit.AMPS) > 2.5) { //current limit end detection
                    reset()
                    resetting = false
                }
            } else {
                if (!motor.isOverCurrent) {
                    val output = pid.update(position)
                    motor.power = sign(output) * sqrt(abs(output))
                } else {
                    motor.power = 0.0
                }
            }
        }

        override fun findZero() {
            motor.power = -0.5
            resetting = true
        }

        var position: Double
            get() = if (!reversed) {
                (encoder.currentPosition * -1) - encoderOffset
            } else {
                encoder.currentPosition - encoderOffset
            }
            set(newPosition) {
                // some weird math happening here
                // this *should* work because we minus in getPosition but unsure
                // TODO: prob causing problems
                encoderOffset = encoder.currentPosition - newPosition
            }


        override fun closeEnough(): Boolean {
            return abs(position - targetPosition) < 20
        }
    }


    class Claw(val servo: Servo, val openPos: Double, val closedPos: Double) {
        var closed: Boolean = false

        init {
            open()
        }


        var position: Double
            get() = servo.position
            set(position) {
                servo.position = position
            }

        fun open() {
            closed = false
            servo.position = openPos
        }

        fun close() {
            closed = true
            servo.position = closedPos
        }

        fun toggle() {
            if (closed) {
                open()
            } else {
                close()
            }
        }
    }

    open class ServoArm(val servo: Servo) {
        var down: Boolean = false
        var upPos: Double = 0.2
        var downPos: Double = 0.03 // TODO TUNE

        init {
            moveDown()
        }

        constructor(servo: Servo, downPos: Double, upPos: Double) : this(servo) {
            this.downPos = downPos
            this.upPos = upPos
        }


        var position: Double
            get() = servo.position
            set(position) {
                servo.position = position
            }

        fun moveUp() {
            down = false
            position = upPos
        }

        fun moveDown() {
            down = true
            position = downPos
        }

        fun toggle() {
            if (down) {
                moveUp()
            } else {
                moveDown()
            }
        }
    }

    class ExtendoArm(servo: Servo) : ServoArm(servo) {
        constructor(servo: Servo, downPos: Double, upPos: Double) : this(servo) {
            this.downPos = downPos
            this.upPos = upPos
        }

        val dumpPos = 0.0 // 0.6
        fun moveDump() {
            down = false
            position = dumpPos
        }
    }

    abstract class ControlledMotor(val motor: DcMotorEx) {
        @JvmField
        var targetPosition = 0.0

        abstract fun update()
        abstract fun reset()
        abstract fun closeEnough(): Boolean
        val isOverCurrent: Boolean
            get() = motor.isOverCurrent

        abstract fun findZero()

        init {
            motors.add(this)
        }
    }

    companion object {
        var motors: ArrayList<ControlledMotor> = ArrayList()
    }
}
