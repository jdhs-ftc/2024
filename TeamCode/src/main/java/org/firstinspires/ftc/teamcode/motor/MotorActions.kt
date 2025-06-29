package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.profile
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.helpers.BetterUniqueAction
import org.firstinspires.ftc.teamcode.helpers.LazyAction
import kotlin.math.abs

class MotorActions(val motorControl: MotorControl) {
    val extendo = Extendo()
    val deposit = Deposit()
    val depositClaw = Claw(motorControl.depositClaw)
    val extendoClaw = Claw(motorControl.extendoClaw)
    val depositArm = DepositArm(motorControl.depositArm)
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
        extendo.setTargetPosition(275.0),
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
            depositClawRelease(), // TODO USE ENCODER
            SleepAction(0.3)

        )
    }

    fun depositScoreChamberTeleop() = SequentialAction (
        deposit.setTargetPosition(900.0), // 1050
        //SleepAction(0.5),
        depositArm.moveDown(),
        SleepAction(0.3),
        depositClawRelease(), // TODO USE ENCODER
        SleepAction(0.1),
        depositClaw.close(),
        SleepAction(0.3),
        depositClawRelease()
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

    fun moveVerticalTransfer() = SequentialAction(
        depositClaw.open(),
        deposit.setTargetPosition(625.0),
        extendo.setTargetPosition(150.0),
        depositArm.setPosition(0.15),
        extendoArm.setPosition(0.8),
    )
    fun grabVerticalTransferReturn() =
        SequentialAction(
            depositClaw.close(),
            SleepAction(0.25), // todo tune
            extendoClaw.open(),
            SleepAction(0.1),
            depositArm.setPosition(0.55),
            SleepAction(0.2),
            deposit.moveUp()
        )

    fun verticalTransferFull(between: Action = SleepAction(0.5)) =
        SequentialAction (
            moveVerticalTransfer(),
            between,
            grabVerticalTransferReturn()
        )

    fun sampleToHighBasketBack() =
        SequentialAction (
            depositArm.setPosition(0.6),
            //SleepAction(0.1),
            depositClaw.open()
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
            return setTargetPosition(1200.0) // prev 1200
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
            SequentialAction (
            Action { return@Action !(encoder.posDegrees < 70) },
                SleepAction(0.1)
            )
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
        open fun setPosition(position: Double): Action {
            return InstantAction { servoArm.position = position }
        }

        open fun moveUp(): Action {
            return InstantAction { servoArm.moveUp() }
        }

        open fun moveDown(): Action {
            return InstantAction { servoArm.moveDown() }
        }
    }

    open class ThreeArm(val threeArm: MotorControl.ThreeArm) : ServoArm(threeArm) {
        open fun moveFullUp(): Action {
            return InstantAction { threeArm.moveFullUp() }
        }
        fun moveDump() = moveFullUp()
        fun moveScore() = moveFullUp()
        fun moveTransfer() = moveUp()
        fun moveMid() = moveFullUp()
    }

    class DepositArm(val depositArm: MotorControl.DepositArm): ThreeArm(depositArm) {
        override fun setPosition(position: Double): Action {
            return LazyAction { goToPosAction(
                start = depositArm.position,
                target = position,
                maxVel = 4.0,
                minAccel = -3.0,
                maxAccel = 4.0,
                resolution = 0.001,
                setPosition = depositArm::position.setter
            ) }
        }
        override fun moveUp(): Action {
            return BetterUniqueAction(BetterUniqueAction(setPosition(depositArm.upPos),"depArmMoveUp",wait=false),"depArm")
        }
        override fun moveDown(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.downPos),
                    "depArmMoveDown",
                    wait = false
                ), "depArm"
            )
        }
        override fun moveFullUp(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.fullUpPos),
                    "depArmMoveFullUp",
                    wait = false
                ), "depArm"
            )
        }
    }
}

fun goToPosAction(
    start: Double,
    target: Double,
    maxVel: Double,
    minAccel: Double,
    maxAccel: Double,
    resolution: Double,
    setPosition: (Double) -> Unit
): Action {
    if (target == start) return Action { false }
    val dispProfile =
        profile(abs(target - start), 0.0, { maxVel }, { minAccel }, { maxAccel }, resolution)
    val profile = TimeProfile(dispProfile.baseProfile)
    val currentTime = ElapsedTime()

    var lastPosition = start
    var lastTime = currentTime.seconds()

    val negative = target < start

    FlightRecorder.write("goToPosAction/start", start)
    FlightRecorder.write("goToPosAction/target", target)
    FlightRecorder.write("goToPosAction/maxVel", maxVel)
    FlightRecorder.write("goToPosAction/minAccel", minAccel)
    FlightRecorder.write("goToPosAction/maxAccel", maxAccel)
    FlightRecorder.write("goToPosAction/resolution", resolution)
    FlightRecorder.write("goToPosAction/negative", negative)


    return SequentialAction( // init loop
        InstantAction {
            currentTime.reset()
            lastTime = currentTime.seconds()
        }, // init
        { // loop
            val time = currentTime.seconds()

            val target = profile[time]
            val targetPos = target[0]
            val targetVel = target[1]
            val targetAccel = target[2]

            val output = if (negative) start - targetPos else start + targetPos

            FlightRecorder.write("goToPosAction/lastPosition", lastPosition)
            FlightRecorder.write("goToPosAction/lastTime", lastTime)
            FlightRecorder.write("goToPosAction/time", time)
            FlightRecorder.write("goToPosAction/targetVel", targetVel)
            FlightRecorder.write("goToPosAction/targetAccel", targetAccel)
            FlightRecorder.write("goToPosAction/output", output)
            setPosition(output)

            lastPosition = output
            lastTime = time
            return@SequentialAction time <= profile.duration
        }
    )
}
