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
    val depositArm = ThreeArm(motorControl.depositArm)
    val extendoArm = ThreeArm(motorControl.extendoArm)
    val depositEncoder = DepositEncoder(motorControl.depositArmEncoder)

    fun waitUntilFinished(): Action {
        return Action { motorControl.closeEnough() }
    }

    fun update(): Action {
        return Action {
            motorControl.update()
            it.put("depositArmPosDegrees", motorControl.depositArmEncoder.posDegrees)
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
            SleepAction(0.3),
            extendoArm.moveUp() // move claw to "clears ground bar" pos
        )
    }

    fun extendoCycle(between: Action = SleepAction(0.8)): Action {
        return SequentialAction(
            extendoClawGround(),
            between, // default wait 0.5
            extendoGrabAndRaise()
        )
    }

    fun depositMoveWall(): Action {
        return SequentialAction(
            deposit.moveDown(),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
            extendo.moveDown(),
            depositArm.moveDown(), // down to intake
            depositClaw.open(),
            Action {!(motorControl.extendo.position < 300) }, // wait for extendo to be retracted, todo tune
            extendoArm.moveFullUp()
        )
    }

    fun depositMoveWallTeleop() = SequentialAction(
        deposit.setTargetPosition(60.0),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
        extendo.setTargetPosition(207.0),
        depositArm.moveDown(), // down to intake
        depositClaw.open(),
        Action { !(motorControl.extendo.position < 300) }, // wait for extendo to be retracted, todo tune
        extendoArm.moveFullUp()
    )

    fun depositPickupWall(): Action {
        return SequentialAction(
        extendoClaw.open(),
        depositClaw.close(),
        SleepAction(0.20), // TODO tune
        depositMoveChamber(),
        //deposit.setTargetPosition(250.0)
        )
    }

    fun depositPickupWallTeleop() : Action {
        return SequentialAction(
            extendoClaw.open(),
            depositClaw.close(),
            SleepAction(0.4), // TODO tune
            depositMoveChamber(),
            //deposit.setTargetPosition(250.0)
        )
    }

    fun depositMoveChamber(): Action {
        return SequentialAction(
            InstantAction { depositArm.threeArm.position = 0.40 },
            deposit.setTargetPosition(300.0), // prev 250 prev 350
        )
    }

    fun depositScoreChamber(): Action {
        return SequentialAction(
            deposit.setTargetPosition(600.0), // 1050
            InstantAction { depositArm.threeArm.position = 0.60 },
            SleepAction(0.1),
            depositClawRelease() // TODO USE ENCODER

        )
    }

    fun depositScoreChamberTeleop() = SequentialAction (
        deposit.setTargetPosition(900.0), // 1050
        //SleepAction(0.5),
        depositArm.moveDown(),
        SleepAction(0.3),
        depositClawRelease() // TODO USE ENCODER
    )
    fun depositClawRelease(): Action {
        return SequentialAction(depositClaw.open())
    }


    fun moveTransfer() =
        SequentialAction(
            deposit.setTargetPosition(117.0),
            extendo.setTargetPosition(634.0),
            depositArm.moveTransfer(),
            InstantAction { depositClaw.claw.position = 0.25 },
            extendoArm.moveFullUp(),
        )

    fun grabTransferReturn() =
        SequentialAction (
            depositClaw.close(),
            SleepAction(0.25), // todo tune
            extendoClaw.open(),
            SleepAction(0.1),
            depositArm.moveDown()
    )

    fun transferFull(between: Action = depositEncoder.waitForTransferGrab()) =
        SequentialAction (
            moveTransfer(),
            between,
            grabTransferReturn()
        )


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
        fun moveTransfer() = setTargetPosition(634.0)
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

    class DepositEncoder(val encoder: MotorControl.AxonEncoder) {
        fun waitForTransferRelease() =
            Action { return@Action !(encoder.posDegrees < 65) }
        fun waitForTransferGrab() =
            Action { return@Action !(encoder.posDegrees > 255) }
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

    class ThreeArm(val threeArm: MotorControl.ThreeArm) : ServoArm(threeArm) {
        fun moveFullUp(): Action {
            return InstantAction { threeArm.moveFullUp() }
        }
        fun moveDump() = moveFullUp()
        fun moveScore() = moveFullUp()
        fun moveTransfer() = moveUp()
    }
}
