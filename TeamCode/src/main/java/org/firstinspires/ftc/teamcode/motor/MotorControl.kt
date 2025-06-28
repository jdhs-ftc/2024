package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.MotorGroup
import org.firstinspires.ftc.teamcode.helpers.RGBLight
import org.firstinspires.ftc.teamcode.helpers.ServoGroup
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt


@Config
class MotorControl(hardwareMap: HardwareMap, lateinit: Boolean = false) {
    @JvmField
    val extendoArm = ThreeArm(hardwareMap.get(Servo::class.java, "sArm"), 0.445, 0.6, 0.85) // 0.425 0.6 0.85 // 0.3 0.6 1.0 //0.03, 0.2) // dump pos 0.6, set in class

    @JvmField
    val extendoClaw = Claw(hardwareMap.get(Servo::class.java, "extendoClaw"), 0.55, 0.387) // might need to be 0.45 0.7???? // 0.32 0.5

    @JvmField
    val extendo: Slide = Slide(
        CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "extendo")), // port 1 of chub and exhub, encoder is left_front
        PIDFController(Constants.extendoPID),
        encoder=Encoder(hardwareMap.get(DcMotorEx::class.java, "left_front"), true),
        reversed=true
    )

    val depositArmServo = ServoGroup(hardwareMap.servo["dArm"],hardwareMap.servo["dArm2"])

    init {
        depositArmServo.setDirections(Servo.Direction.FORWARD, Servo.Direction.REVERSE)
        depositArmServo.scaleRange(0, Constants.depArmS1min, Constants.depArmS1max)
        depositArmServo.scaleRange(1, Constants.depArmS2min, Constants.depArmS2max)
    }

    @JvmField
    val depositArm = ThreeArm(depositArmServo, 0.4, 0.97, 0.5)

    @JvmField
    val depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.30, 0.1) // 0.35 0.1 // 0.55 0.1

    val depositMotors = MotorGroup(
        hardwareMap.get(DcMotorEx::class.java, "deposit1"),
        hardwareMap.get(DcMotorEx::class.java, "deposit2")
    )
    init {
        depositMotors.setDirections(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD)
    }

    @JvmField
    val deposit: Slide = Slide(
        CachingDcMotorEx(
            depositMotors
        ),
        // port 0 of exp hub and 0 of chub,
        // encoder is left_back
        PIDFController(
            Constants.depositPID
        ), // { a, b -> 0.05 }, // Static feedforward
        encoder=Encoder(hardwareMap.get(DcMotorEx::class.java, "right_front"), true),
        reversed=false
    )

    val dColor = BLColor(hardwareMap.digitalChannel["digital0"],hardwareMap.digitalChannel["digital1"])

    val depositArmEncoder = AxonEncoder(hardwareMap.analogInput["depositArmEncoder"])

    val extendoArmEncoder = AxonEncoder(hardwareMap.analogInput["extendoArmEncoder"])

    val topLight = RGBLight(hardwareMap.servo.get("rgb"))

    val motors = listOf(extendo, deposit)

    init {
        if (!lateinit) {
            init()
        }
    }

    fun init() {
        topLight.color = RGBLight.Color.YELLOW
        depositArm.moveDown()
        extendoArm.moveUp()

        extendoClaw.open()
        depositClaw.open()

        extendoArm.moveFullUp()

        motors.forEach { it.findZero() }
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    fun update() {
        motors.forEach { it.update() }

        FlightRecorder.write("MotorControl/extendoArm/target", extendoArm.position)
        FlightRecorder.write("MotorControl/extendoArm/actual", extendoArmEncoder.position)
        FlightRecorder.write("MotorControl/depositArm/target", depositArm.position)
        FlightRecorder.write("MotorControl/depositArm/actual", depositArmEncoder.position)
        FlightRecorder.write("MotorControl/deposit/target", deposit.targetPosition)
        FlightRecorder.write("MotorControl/deposit/actual", deposit.position)
        FlightRecorder.write("MotorControl/extendo/target", extendo.targetPosition)
        FlightRecorder.write("MotorControl/extendo/actual", extendo.position)
        FlightRecorder.write("MotorControl/dColor/color", dColor.color)
        FlightRecorder.write("MotorControl/extendoClaw/target", extendoClaw.position)
        FlightRecorder.write("MotorControl/depositClaw/target", depositClaw.position)
        FlightRecorder.write("MotorControl/topLight/color", topLight.servo.position)

        if (topLight.color == RGBLight.Color.YELLOW && motors.all { !it.resetting }) {
            topLight.color = RGBLight.Color.GREEN
        }
    }


    fun closeEnough(): Boolean {
        return motors.all { it.closeEnough() }
    }

    val isOverCurrent: Boolean
        get() = motors.any { it.isOverCurrent }

    class AxonEncoder(val pin: AnalogInput) {
        val position
            get() = pin.voltage / pin.maxVoltage

        val posDegrees
            get() = pin.voltage / pin.maxVoltage * 360
    }

    class BLColor(val pin0: DigitalChannel, val pin1: DigitalChannel) {
        enum class Color(val first: Boolean, val second: Boolean) {
            NONE(false, false),
            RED(false, true),
            BLUE(true, false),
            YELLOW(true, true);


            companion object {
                fun get(first: Boolean, second: Boolean): Color {
                    return if (first) {
                        if (second) {
                            YELLOW
                        } else {
                            BLUE
                        }
                    } else if (second) {
                        RED
                    } else {
                        NONE
                    }
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

    class Encoder(val motor: DcMotor, val reversed: Boolean = false) {
        private var offset = 0.0

        var position: Double
            get() = if (!reversed) {
                (motor.currentPosition * -1) - offset
            } else {
                motor.currentPosition - offset
            }
            set(newPosition) {
                // some weird math happening here
                // this *should* work because we minus in getPosition but unsure
                // TODO: prob causing problems
                offset = motor.currentPosition - newPosition
            }

        fun reset() {
            position = 0.0
        }
    }

    class Slide(motor: DcMotorEx, val pid: PIDFController, val encoder: Encoder, val reversed: Boolean = false) :
        ControlledMotor(motor) {
        var resetting = false

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
            get() = encoder.position
            set(position) { encoder.position = position }


        override fun closeEnough(): Boolean {
            return abs(position - targetPosition) < 25
        }
    }


    class Claw(val servo: Servo, val openPos: Double, val closedPos: Double) {
        var closed: Boolean = false

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
