package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction

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


    fun log(message: String): Action {
        return InstantAction { println(message) }
    }

    // presets
    fun extendoClawGround(): Action {
        return SequentialAction(
            extendoClaw.open(), // open claw
            extendoArm.moveDown(), // move to ground
        )
    }

    fun extendoGrabAndRaise(): Action {
        return SequentialAction(
            extendoClaw.close(), // close claw
            SleepAction(0.5),
            extendoArm.moveUp() // move claw to "clears ground bar" pos
        )
    }

    fun extendoCycle(between: Action = SleepAction(0.5)): Action {
        return SequentialAction(
            extendoClawGround(),
            between,
            extendoGrabAndRaise()
        )
    }

    fun depositMoveWall(): Action {
        return SequentialAction(
            deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
            extendo.moveDown(),
            extendoArm.moveDump(),
            depositArm.moveUp(),
            depositClaw.open()
        )
    }

    fun depositPickupWall(): Action {
        return SequentialAction(
        extendoClaw.open(),
        depositClaw.close(),
        SleepAction(0.5), // TODO tune
        deposit.setTargetPosition(250.0)
        )
    }


    inner class Extendo {
        fun setTargetPosition(position: Double): Action {
            return InstantAction {
                motorControl.extendo.targetPosition = position
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
            return setTargetPosition(1100.0) // prev 1200
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
