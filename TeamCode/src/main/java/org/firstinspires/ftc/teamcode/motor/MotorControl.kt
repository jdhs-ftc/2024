package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.CachingDcMotorEx
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
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
    val extendoArm = ThreeArm(hardwareMap.get(Servo::class.java, "sArm"), 0.99, 0.89, 0.95) // DOWN UP MID

    val intake = Intake(hardwareMap.get(CRServo::class.java, "intake"))


    val extendoMotors = MotorGroup(
        hardwareMap.get(DcMotorEx::class.java, "extendo1"),
        hardwareMap.get(DcMotorEx::class.java, "extendo2")
    )

    init {
        extendoMotors.setDirections(
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD
        )
    }

    @JvmField
    val extendo: Slide = Slide(
        CachingDcMotorEx(extendoMotors), // port 1 of chub and exhub, encoder is left_front
        PIDFController(Constants.extendoPID),
        encoder=Encoder(hardwareMap.get(DcMotorEx::class.java, "left_front"), true),
        reversed=false
    )

    val depositArmServo = ServoGroup(hardwareMap.servo["dArm"],hardwareMap.servo["dArm2"])

    init {
        depositArmServo.setDirections(Servo.Direction.FORWARD, Servo.Direction.REVERSE)
        depositArmServo.scaleRange(0, Constants.depArmS1min, Constants.depArmS1max)
        depositArmServo.scaleRange(1, Constants.depArmS2min, Constants.depArmS2max)
    }

    val depositArmEncoderInput: AnalogInput = hardwareMap.analogInput["depositArmEncoder"]

    val depositArmEncoder = AxonEncoder { (3.3 - depositArmEncoderInput.voltage) * 1.05 }

    @JvmField
    val depositArm = DepositArm(depositArmServo, 0.4, 0.96, 0.72, depositArmEncoder)

    @JvmField
    val depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.79, 0.48) // 0.35 0.1 // 0.55 0.1

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

    val eColor = BLColor(hardwareMap.digitalChannel["digital2"],hardwareMap.digitalChannel["digital3"])


    val extendoArmEncoder = AxonEncoder(hardwareMap.analogInput["extendoArmEncoder"])

    val topLight = RGBLight(hardwareMap.servo.get("rgb"))

    val motors = listOf(extendo, deposit)

    var resetting = true

    init {
        if (!lateinit) {
            init()
        }
    }
    
    val logger = LogTelemetry("MotorControl/")

    fun init() {
        topLight.color = Color.YELLOW
        val depositArmStart = depositArmEncoder.position
        if (depositArmEncoder.position > 0.5) {
            // encoder isn't perfect
            // but this will make it not slam
            // and then the mp can deal with it
            depositArm.position = depositArmEncoder.position
        } else {
            depositArm.position = 0.4
        }
        extendoArm.moveDown()

        intake.stop()
        depositClaw.open()

        extendoArm.moveMid()

        motors.forEach { it.findZero() }
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    fun update() {
        motors.forEach { it.update() }

        logger.write("extendoArm/target", extendoArm.position)
        logger.write("extendoArm/actual", extendoArmEncoder.position)
        logger.write("depositArm/target", depositArm.position)
        logger.write("depositArm/actual", depositArmEncoder.position)
        logger.write("deposit/target", deposit.targetPosition)
        logger.write("deposit/actual", deposit.position)
        logger.write("extendo/target", extendo.targetPosition)
        logger.write("extendo/actual", extendo.position)
        logger.write("dColor/color", dColor.color)
        logger.write("eColor/color", eColor.color)
        logger.write("intake/power", intake.servo.power)
        logger.write("depositClaw/target", depositClaw.position)
        logger.write("topLight/colorPos", topLight.servo.position)
        logger.write("topLight/color", topLight.color)

        logger.update()


        if (resetting && motors.none { it.resetting }) {
            topLight.color = Color.GREEN
            resetting = false
        }
    }


    fun closeEnough(): Boolean {
        return motors.all { it.closeEnough() }
    }

    val isOverCurrent: Boolean
        get() = motors.any { it.isOverCurrent }

    class AxonEncoder(val getter: () -> Double) {
        constructor(pin: AnalogInput): this(pin::getVoltage)
        val position
            get() = getter() / 3.3

        val posDegrees
            get() = getter() / 3.3 * 360
    }

    class BLColor(val pin0: DigitalChannel, val pin1: DigitalChannel) {
        fun boolsToColor(first: Boolean, second: Boolean): Color {
            return if (first) {
                if (second) {
                    Color.YELLOW
                } else {
                    Color.BLUE
                }
            } else if (second) {
                Color.RED
            } else {
                Color.NONE
            }
        }
        val color: Color
            get() {
                // this could be EXTREMELY inefficient and trigger a read,
                // but as long as you remember to use bulk reads, it's fine :)
                return boolsToColor(pin0.state, pin1.state)
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

    open class ThreeArm(servo: Servo,
                   downPos: Double = 0.03, //wrong
                   upPos: Double = 0.2, // wrong
                   val midPos: Double = 0.6) // tuned
        : ServoArm(servo, downPos,upPos) {
        val mid: Boolean
            get() {
                return abs(position -  midPos) < 0.05
            }

        fun moveMid() {
            position = midPos
        }
    }

    class DepositArm(servo: Servo, downPos: Double, upPos: Double, midPos: Double, encoder: AxonEncoder): ThreeArm(servo, downPos, upPos, midPos) {
        fun moveForward() = moveUp()
        fun moveBack() = moveDown()
        fun moveHang() = moveMid()
    }


    class Intake(val servo: CRServo, val inSpeed: Double = 1.0, val outSpeed: Double = -1.0) {
        fun intake() {
            servo.power = inSpeed
        }

        fun eject() {
            servo.power = outSpeed
        }

        fun stop() {
            servo.power = 0.0
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

    }
}
