package org.firstinspires.ftc.teamcode.motor

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController
import org.firstinspires.ftc.teamcode.helpers.control.PIDFController.PIDCoefficients
import kotlin.math.abs


/**
 * This class is used to control the motor systems on the robot.
 */
class MotorControl(hardwareMap: HardwareMap) {
    init { //empty out the motors list, its static so remains between op modes by default :skull:
        motors = ArrayList()
    }
    @JvmField
    val extendoArm: ServoArm
    @JvmField
    val extendoClaw: Claw
    @JvmField
    val extendo: Slide = Slide(
        hardwareMap,  // port 0 of exp hub and chub, equiv to left_back I think
        "extendo",
        PIDFController(PIDCoefficients(0.01, 0.0, 0.0))
    )
    @JvmField
    val depositArm: ServoArm
    @JvmField
    val depositClaw: Claw
    @JvmField
    val deposit: Slide
    //public final ColorSensor color;
    init {
        extendo.encoder = hardwareMap.get(DcMotorEx::class.java, "left_front")
        extendo.reversed = true
        deposit = Slide(
            hardwareMap,  // port 1 of exp hub and chub,
            // equiv to left_front I think could be wrong though
            "deposit",
            PIDFController(PIDCoefficients(0.01, 0.0, 0.0), PIDFController.FeedforwardFun{a,b -> return@FeedforwardFun 0.15 })
        )
        deposit.encoder = hardwareMap.get(DcMotorEx::class.java, "left_back")
        deposit.reversed = true


        extendoClaw = Claw(hardwareMap.get(Servo::class.java, "extendoClaw"), 0.7, 1.0)
        depositClaw = Claw(hardwareMap.get(Servo::class.java, "depositClaw"), 0.0, 1.0)
        extendoArm = ExtendoArm(hardwareMap.get(Servo::class.java, "sArm"), 0.03, 0.2) // dump pos 0.6, set in class
        depositArm = ServoArm(hardwareMap.get(Servo::class.java, "sArm"), 0.0, 1.0) // TODO: CHANGE TO THE RIGHT ONE

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
    class Slide(hardwareMap: HardwareMap, motorName: String, val pid: PIDFController) :
        ControlledMotor(hardwareMap.get(DcMotorEx::class.java, motorName)) {
        var reversed = true // TODO EVERYTHING REVERSED
        var resetting = false
        var encoder: DcMotorEx = motor
        var encoderOffset: Double = 0.0

        init {
            motor.zeroPowerBehavior =DcMotor .ZeroPowerBehavior.BRAKE
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
                    motor.power = pid.update(position) // TODO SQRT
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
                (encoder.currentPosition * -1) - encoderOffset
            } else {
                encoder.currentPosition - encoderOffset
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
        val dumpPos = 0.6
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
