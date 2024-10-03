package org.firstinspires.ftc.teamcode.motor

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import kotlin.math.abs


/**
 * This class is used to control the motor systems on the robot.
 */
class MotorControl(hardwareMap: HardwareMap) {
    @JvmField
    val sArm: ServoArm
    @JvmField
    val extendoClaw: Claw
    @JvmField
    val depositClaw: Claw
    @JvmField
    val extendo: Slide = Slide(
        hardwareMap,  // port 0 of exp hub and chub, equiv to left_back I think
        "extendo",
        PIDFController.PIDCoefficients(0.01, 0.0, 0.0)
    )

    @JvmField
    val deposit: Slide
    //public final ColorSensor color;
    init {
        extendo.setEncoder(hardwareMap.get(DcMotorEx::class.java, "left_front"))
        extendo.reversed = true
        deposit = Slide(
            hardwareMap,  // port 1 of exp hub and chub,
            // equiv to left_front I think could be wrong though
            "deposit",
            PIDFController.PIDCoefficients(0.01, 0.0, 0.0)
        )
        deposit.setEncoder(hardwareMap.get(DcMotorEx::class.java, "left_back"))
        deposit.reversed = true


        extendoClaw = Claw(hardwareMap.get(Servo::class.java, "extendoClaw"), 0.7, 1.0)
        depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.0, 1.0);
        sArm = ServoArm(hardwareMap.get(Servo::class.java, "sArm"))

        //color = hardwareMap.get(ColorSensor.class, "color");
        for (motor in motors) {
            motor.findZero()
        }
    }

    /**
     * This class updates the arm and slide motors to match the current state.
     */
    fun update() {
        for (motor in motors) {
            motor.update()
        }
    }


    fun closeEnough(): Boolean {
        return motors.stream().allMatch { obj: ControlledMotor -> obj.closeEnough() }
    }

    val isOverCurrent: Boolean
        get() = motors.stream().anyMatch { obj: ControlledMotor -> obj.isOverCurrent }

    /**
     * This class controls the slide motor.
     */
    class Slide(hardwareMap: HardwareMap, motorName: String?, pidCoefficients: PIDFController.PIDCoefficients?) :
        ControlledMotor(hardwareMap.get(DcMotorEx::class.java, motorName)) {
        var reversed: Boolean = true // TODO EVERYTHING REVERSED
        var resetting: Boolean = false
        var pid: PIDFController = PIDFController(pidCoefficients)
        var encoder: DcMotorEx?
        var encoderOffset: Double = 0.0

        init {
            encoder = motor
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.setCurrentAlert(6.0, CurrentUnit.AMPS)
            if (reversed) {
                motor.direction = DcMotorSimple.Direction.REVERSE
            } else {
                motor.direction = DcMotorSimple.Direction.FORWARD
            }
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        fun setEncoder(motor: DcMotorEx?) {
            encoder = motor
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
                    motor.power = pid.update(position)
                } else {
                    motor.power = 0.0
                }
            }
        }

        override fun findZero() {
            motor.power = -0.5
            resetting = true
        }

        var position: Double = 0.0
            get() = if (!reversed) {
                (encoder!!.currentPosition * -1) - encoderOffset
            } else {
                encoder!!.currentPosition - encoderOffset
            }
            set(position) {
                // some weird math happening here
                // this *should* work because we minus in getPosition but unsure
                // TODO: prob causing problems
                encoderOffset += field
                field = position
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

    class ServoArm(val servo: Servo) {
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
            servo.position = upPos
        }

        fun moveDown() {
            down = true
            servo.position = downPos
        }

        fun toggle() {
            if (down) {
                moveUp()
            } else {
                moveDown()
            }
        }
    }

    abstract class ControlledMotor(val motor: DcMotorEx) {
        @JvmField
        var targetPosition: Double = 0.0

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
