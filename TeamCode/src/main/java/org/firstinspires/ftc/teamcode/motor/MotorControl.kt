package org.firstinspires.ftc.teamcode.motor

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController.PIDCoefficients
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt


/**
 * This class is used to control the motor systems on the robot.
 */
class MotorControl(hardwareMap: HardwareMap) {

    @JvmField
    val extendoArm = ThreeArm(hardwareMap.get(Servo::class.java, "sArm"), 0.445, 0.6, 0.85) // 0.425 0.6 0.85 // 0.3 0.6 1.0 //0.03, 0.2) // dump pos 0.6, set in class

    @JvmField
    val extendoClaw = Claw(hardwareMap.get(Servo::class.java, "extendoClaw"), 0.55, 0.387) // might need to be 0.45 0.7???? // 0.32 0.5

    @JvmField
    val extendo: Slide = Slide(
        CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "extendo")), // port 0, encoder is left_front
        PIDFController(PIDCoefficients(0.001, 0.0, 0.0)),
        encoder=hardwareMap.get(DcMotorEx::class.java, "left_front"),
        reversed=true
    )

    @JvmField
    val depositArm = ThreeArm(hardwareMap.get(Servo::class.java, "dArm"), 0.97, 0.265, 0.5) // TODO TUNE

    @JvmField
    val depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.30, 0.1) // 0.35 0.1 // 0.55 0.1

    @JvmField
    val deposit: Slide = Slide(
        CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "deposit")),
        // port 1 of exp hub and chub,
        // encoder is left_back
        PIDFController(
            PIDCoefficients(0.005, 0.0, 0.0),
            PIDFController.FeedforwardFun { a, b -> return@FeedforwardFun 0.05 }),
        encoder=hardwareMap.get(DcMotorEx::class.java, "left_back"),
        reversed=true
    )

    val dColor = BLColor(hardwareMap.digitalChannel["digital0"],hardwareMap.digitalChannel["digital1"])

    val depositArmEncoder = AxonEncoder(hardwareMap.analogInput["depositArmEncoder"])

    val extendoArmEncoder = AxonEncoder(hardwareMap.analogInput["extendoArmEncoder"])

    val motors = listOf(extendo, deposit)

    //public final ColorSensor color;
    init {
        depositArm.moveDown()
        extendoArm.moveUp()

        motors.forEach { it.findZero() }

        extendoArm.moveFullUp()
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

    class AxonEncoder(val pin: AnalogInput) {
        val position
            get() = pin.voltage / pin.maxVoltage

        val posDegrees
            get() = pin.voltage / pin.maxVoltage * 360
    }

    class BLColor(val pin0: DigitalChannel, val pin1: DigitalChannel) {
        enum class Color(val first: Boolean, val second: Boolean) {
            YELLOW(true, true),
            RED(false, true),
            BLUE(true, false),
            NONE(false, false);

            companion object {
                fun get(first: Boolean, second: Boolean): Color {
                    // this is somewhat goofy and inefficient
                    // I should find a better way of doing this,
                    // im sure there's some elegant kotlin one, but too sleepy for that
                    return Color.entries.stream().filter { return@filter it.first == first && it.second == second }.findFirst().get()
                }
            }
        }
        val color: Color
            get() {
                // this could be EXTREMELY inefficient and trigger a read,
                // but as long as you remember to use bulk reads, it's fine :)
                return Color.get(pin0.state, pin1.state)
            }

    }

    class Slide(motor: DcMotorEx, val pid: PIDFController, val encoder: DcMotorEx = motor, val reversed: Boolean = true) :
        ControlledMotor(motor) {
        var resetting = false

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
                    //motor.power = output
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
            return abs(position - targetPosition) < 25
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
        val down: Boolean
            get() {
                return position == downPos
            }
        var upPos: Double = 0.2
        var downPos: Double = 0.03 // TODO TUNE

        init {
            moveDown()
        }

        constructor(servo: Servo, downPos: Double, upPos: Double) : this(servo) {
            this.downPos = downPos
            this.upPos = upPos
        }


        var position: Double = 0.0 // cache last sent value to read from
            set(position) {
                servo.position = position
                field = position
            }

        fun moveUp() {
            position = upPos
        }

        fun moveDown() {
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

    class ThreeArm(servo: Servo,
                   downPos: Double = 0.03, //wrong
                   upPos: Double = 0.2, // wrong
                   val fullUpPos: Double = 0.6) // tuned
        : ServoArm(servo, downPos,upPos) {
        val fullyUp: Boolean // AKA dumping
            get() {
                return abs(position -  fullUpPos) < 0.05
            }
        init {
            moveFullUp()
        }
        fun moveFullUp() { // AKA moveDump
            position = fullUpPos
        }

        fun moveDump() = moveFullUp()
        fun moveScore() = moveFullUp()
        fun moveTransfer() = moveUp()
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

    }
}
