package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.profile
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.helpers.BetterUniqueAction
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.LazyAction
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.RepeatUntilAction
import kotlin.math.abs

class MotorActions(val motorControl: MotorControl) {
    val extendo = Extendo()
    val deposit = Deposit()
    val depositClaw = Claw(motorControl.depositClaw)
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

    fun intakeUntilColor() = SequentialAction(
        RepeatUntilAction(
        { motorControl.eColor.color == PoseStorage.currentTeam.color },
        {
            SequentialAction(
                InstantAction { motorControl.intake.intake() },
                { motorControl.eColor.color == Color.NONE }, // repeat until ANY sample detected
                InstantAction { motorControl.intake.stop() },
                SleepAction(0.1),
                // check condition
            )
        }
    ), SequentialAction (
            InstantAction {motorControl.topLight.color = motorControl.eColor.color}, // TODO intake light?
            intakeIn()
    ))

    fun intakeIn() = SequentialAction(
        extendoArm.moveIn(), // TODO time??
        extendo.moveDown(),
    )

    fun intakeOut() = SequentialAction(
        extendo.moveUp(),
        Action { !(motorControl.extendo.position > 300) }, // TODO tune
        InstantAction { motorControl.intake.eject() }, // TODO see if this even makes sesne
        extendoArm.moveOut(),
        SleepAction(0.3),
        InstantAction { motorControl.intake.stop() }
    )

    fun intakeInOut(condition: () -> Boolean = { false }) =
        SequentialAction (
            intakeIn(),
            { !condition() },
            intakeOut()
    )



    fun allIn() = ParallelAction (
        intakeIn(),
        depositArm.moveDown(),
        deposit.moveDown()
    )


    fun depositMoveWall(): Action {
        return SequentialAction(
            deposit.moveDown(),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
            extendo.moveDown(),
            depositArm.moveDown(), // down to intake
            depositClaw.open(),
            Action { !(motorControl.extendo.position < 300) }, // wait for extendo to be retracted, todo tune
            extendoArm.moveMid()
        )
    }

    fun depositMoveWallTeleop() = SequentialAction(
        deposit.setTargetPosition(60.0),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
        extendo.setTargetPosition(275.0),
        depositArm.moveDown(), // down to intake
        depositClaw.open(),
        Action { !(motorControl.extendo.position < 300) }, // wait for extendo to be retracted, todo tune
        extendoArm.moveMid()
    )

    fun depositPickupWall(): Action {
        return SequentialAction(
            depositClaw.close(),
            SleepAction(0.20), // TODO tune
            depositMoveChamber(),
            //deposit.setTargetPosition(250.0)
        )
    }

    fun depositPickupWallTeleop(): Action {
        return SequentialAction(
            depositClaw.close(),
            SleepAction(0.4), // TODO tune
            depositMoveChamber(),
            //deposit.setTargetPosition(250.0)
        )
    }

    fun depositMoveChamber() = depositMoveChamberFar()
    fun depositScoreChamber() = depositScoreChamberFar()

    fun depositMoveChamberFar(): Action {
        return SequentialAction(
            depositClaw.close(),
            SleepAction(0.1),
            depositArm.moveDown(),
            deposit.setTargetPosition(1457.0), //
        )
}

    fun depositScoreChamberFar(): Action {
        return SequentialAction(
            //deposit.setTargetPosition(1110.0), // 1050
            depositArm.setPosition(0.5),
            /*
            SleepAction(0.1),
            depositClawRelease(), // TODO USE ENCODER
            SleepAction(0.3),
            deposit.moveDown(),
            depositArm.moveUp(),

             */
        )
    }

    fun depositMoveChamberAligned(): Action {
        return SequentialAction(
            depositClaw.close(),
            SleepAction(0.1),
            depositArm.moveDown(),
            deposit.setTargetPosition(590.0), //
        )
    }

    fun depositScoreChamberAligned(): Action {
        return SequentialAction(
            deposit.setTargetPosition(590.0), // 1050
            depositArm.setPosition(0.55),
            SleepAction(0.1),
            depositClawRelease(), // TODO USE ENCODER
            SleepAction(0.3),
            deposit.moveDown(),
            depositArm.moveUp(),
        )
    }

    fun depositScoreChamberTeleop() = depositScoreChamber()
        /*
        SequentialAction(
        deposit.setTargetPosition(900.0), // 1050
        //SleepAction(0.5),
        depositArm.moveHang(),
        SleepAction(0.3),
        depositClawRelease(), // TODO USE ENCODER
        SleepAction(0.3),
        depositArm.moveUp()
    )*/

    fun depositClawRelease(): Action {
        return SequentialAction(depositClaw.open())
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
            SequentialAction(
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

        fun moveIn() = moveDown()
        fun moveOut() = moveUp()
    }

    open class ThreeArm(val threeArm: MotorControl.ThreeArm) : ServoArm(threeArm) {
        open fun moveMid(): Action {
            return InstantAction { threeArm.moveMid() }
        }

        fun moveDump() = moveMid()
        fun moveScore() = moveMid()
    }

    class DepositArm(val depositArm: MotorControl.DepositArm) : ThreeArm(depositArm) {
        override fun setPosition(position: Double): Action {
            return LazyAction {
                goToPosAction(
                    start = depositArm.position,
                    target = position,
                    maxVel = 4.0,
                    minAccel = -3.0,
                    maxAccel = 4.0,
                    resolution = 0.01,
                    setPosition = depositArm::position.setter
                )
            }
        }

        override fun moveUp(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.upPos),
                    "depArmMoveUp",
                    wait = false
                ), "depArm"
            )
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

        override fun moveMid(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.midPos),
                    "depArmMoveFullUp",
                    wait = false
                ), "depArm"
            )
        }

        fun moveHang() = moveMid()
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
