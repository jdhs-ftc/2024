package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import org.firstinspires.ftc.teamcode.motor.MotorActions.Deposit
import org.firstinspires.ftc.teamcode.motor.MotorActions.Extendo

class MotorActions(val motorControl: MotorControl) {
    val extendo = Extendo()
    val deposit = Deposit()
    val depositClaw = Claw(motorControl.depositClaw)
    val extendoClaw = Claw(motorControl.extendoClaw)
    val depositArm = ServoArm(motorControl.depositArm)
    val extendoArm = ExtendoArm(motorControl.extendoArm)
    val depositLid = Claw(motorControl.depositLid)

    fun waitUntilFinished(): Action {
        return Action { t: TelemetryPacket? -> motorControl.closeEnough() }
    }

    fun update(): Action {
        return Action { t: TelemetryPacket? ->
            motorControl.update()
            true // this returns true to make it loop forever; use RaceParallelCommand
        }
    }


    fun log(message: String?): Action {
        return object : Action {
            override fun run(t: TelemetryPacket): Boolean {
                println(message)
                return false
            }
        }
    }


    inner class Extendo {
        fun setTargetPosition(position: Double): Action {
            return Action { t: TelemetryPacket? ->
                motorControl.extendo.targetPosition = position
                false
            }
        }

        fun waitUntilFinished(): Action {
            return object : Action {
                override fun run(t: TelemetryPacket): Boolean {
                    return !motorControl.extendo.closeEnough()
                }
            }
        }

        fun moveUp(): Action {
            return setTargetPosition(1200.0)
        }

        fun moveDown(): Action {
            return setTargetPosition(40.0)
        }
    }

    inner class Deposit {
        fun setTargetPosition(position: Double): Action {
            return InstantAction {
                motorControl.deposit.targetPosition = position
            }
        }

        fun waitUntilFinished(): Action {
            return Action {
                return@Action !motorControl.deposit.closeEnough()
            }
        }

        fun moveUp(): Action {
            return setTargetPosition(1600.0)
        }

        fun moveDown(): Action {
            return setTargetPosition(20.0)
        }
    }

    class Claw internal constructor(val claw: MotorControl.Claw) {

        // TODO: add waits? depends on delay
        fun close(): Action {
            return InstantAction(InstantFunction { claw.close() })
        }

        fun open(): Action {
            return InstantAction(InstantFunction { claw.open() })
        }
    }

    open class ServoArm(val servoArm: MotorControl.ServoArm) {
        fun setPosition(position: Double): Action {
            return InstantAction { servoArm.position = position }
        }

        fun moveUp(): Action {
            return InstantAction { servoArm.moveUp() }
        }

        fun moveDown(): Action {
            return InstantAction { servoArm.moveDown() }
        }
    }

    class ExtendoArm(val extendoArm: MotorControl.ExtendoArm) : ServoArm(extendoArm) {
        fun moveDump(): Action {
            return InstantAction { extendoArm.moveDump() }
        }
    }
}
