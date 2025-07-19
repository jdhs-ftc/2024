package org.firstinspires.ftc.teamcode.motor

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.profile
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.helpers.BetterUniqueAction
import org.firstinspires.ftc.teamcode.helpers.Color
import org.firstinspires.ftc.teamcode.helpers.ForeverAction
import org.firstinspires.ftc.teamcode.helpers.IfAction
import org.firstinspires.ftc.teamcode.helpers.InterruptibleAction
import org.firstinspires.ftc.teamcode.helpers.LazyAction
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.helpers.RepeatUntilAction
import java.lang.Math.toDegrees
import kotlin.math.abs

class MotorActions(val motorControl: MotorControl) {
    val logger = LogTelemetry("MotorActions/")


    val extendo = Extendo()
    val deposit = Deposit()
    val depositClaw = Claw(motorControl.depositClaw)
    val depositArm = DepositArm(motorControl.depositArm, logger)
    val extendoArm = ThreeArm(motorControl.extendoArm)
    val depositEncoder = DepositEncoder(motorControl.depositArmEncoder)


    fun waitUntilFinished(): Action {
        return Action { motorControl.closeEnough() }
    }

    fun autoUpdate(): Action {
        return ParallelAction(Action {
            motorControl.update()
            logger.update() // double updating is unnecessary but whatever
            it.put("depositArmPosDegrees", motorControl.depositArmEncoder.posDegrees)
            true // this returns true to make it loop forever; use RaceParallelCommand
        }, ForeverAction {
            SequentialAction(
                InstantAction { motorControl.topLight.color = Color.BLUE },
                SleepAction(4.0),
                InstantAction { motorControl.topLight.color = Color.GREEN },
                SleepAction(4.0),
            )
        })
    }


    fun log(key: String, message: Any): Action = InstantAction { logger.addData(key, message) }

    fun intakeUntilColor(gamepad1: Gamepad = Gamepad(), gamepad2: Gamepad = Gamepad()) =
        InterruptibleAction(
            SequentialAction(
                RepeatUntilAction({ motorControl.eColor.color == PoseStorage.currentTeam.color }, {
                    SequentialAction(
                        extendoArm.moveDown(),
                        InstantAction { motorControl.topLight.color = Color.GREEN },
                        log("/intakeUntilColor/status", "Started 64"),
                        InstantAction { motorControl.intake.intake() },
                        log("/intakeUntilColor/status", "Ejecting 66"),
                        { motorControl.eColor.color != Color.NONE }, // repeat until sample cleared
                        log("/intakeUntilColor/status", "Sample Clear 68"),
                        SleepAction(0.25), // and a little longer then that TODO tune
                        log("/intakeUntilColor/status", "Sleep Done 70"),
                        InstantAction { motorControl.intake.intake() },
                        log("/intakeUntilColor/status", "Intaking 72"),
                        { motorControl.eColor.color == Color.NONE }, // repeat until ANY sample detected
                        log("/intakeUntilColor/status", "Stopped 74"),
                        InstantAction { motorControl.intake.stop() },
                        SleepAction(0.1),
                        // check condition
                    )
                }), SequentialAction(
                    log("/intakeUntilColor/status", "Done 81"), InstantAction {
                        motorControl.topLight.color = motorControl.eColor.color
                    }, // TODO intake light?
                    InstantAction {
                        gamepad1.rumbleBlips(3)
                        gamepad2.rumbleBlips(3)
                    }, intakeIn()
                )
            )
        )

    fun intakeIn() = SequentialAction(
        extendoArm.moveUp(), // TODO time??
        extendo.moveDown(),
    )

    fun intakeOut() = ParallelAction(
        depositArm.moveDown(), SequentialAction(
            extendo.moveUp(),
            /*
            Action { !(motorControl.extendo.position > 300) }, // TODO tune
            InstantAction { motorControl.intake.eject() }, // TODO see if this even makes sesne
            extendoArm.moveDown(),
            SleepAction(0.3),
            InstantAction { motorControl.intake.stop() }

             */
        )
    )

    fun intakePresetStart(extendoPosition: Double = 800.0) = ParallelAction(
        extendoArm.moveDown(),
        extendo.setTargetPosition(extendoPosition),
        InstantAction { motorControl.intake.intake() }
    )

    fun intakePresetFinish() = SequentialAction(

        RaceParallelAction(
            { motorControl.eColor.color == Color.NONE },
            SleepAction(1.0)
        ),
        InstantAction { motorControl.intake.stop() },
        ParallelAction(
            extendoArm.moveMid(), extendo.moveDown()
        ),
        SleepAction(0.1)
    )

    fun intakePreset() = SequentialAction(intakePresetStart(), intakePresetFinish())

    fun intakeAutoHpEject(drive: MecanumDrive) = SequentialAction(
        IfAction(
            { motorControl.eColor.color == Color.NONE },
            SequentialAction(
                InstantAction { motorControl.intake.intake() },
                SleepAction(0.25),
                InstantAction { motorControl.intake.stop() })

        ), ParallelAction(

            SequentialAction(
                /*
                {
                    logger.write("poseHeadingDeg", toDegrees(drive.pose.heading.log()));
                    !(toDegrees(drive.pose.heading.log()) in -160.0..-20.0)
                },

             */
                SleepAction(0.25),
                depositMoveWall(),
            ), SequentialAction(
                {
                    logger.write("poseHeadingDeg", toDegrees(drive.pose.heading.log()));
                    !(toDegrees(drive.pose.heading.log()) in -120.0..-70.0)
                },
                //extendoArm.moveDown(),
                extendo.setTargetPosition(700.0),
                //depositMoveWall(),
                SleepAction(0.1), InstantAction { motorControl.intake.eject() }, RaceParallelAction(
                    { motorControl.eColor.color != Color.NONE }, SleepAction(2.0)
                ), SleepAction(0.25), InstantAction { motorControl.intake.stop() }, ParallelAction(
                    //extendoArm.moveMid(),
                    extendo.moveDown()
                )
            )
        )
    )

    fun intakeInOut(condition: () -> Boolean = { false }) = SequentialAction(
        intakeIn(), { !condition() }, intakeOut()
    )


    fun allIn() = ParallelAction(
        intakeIn(), depositArm.moveDown(), deposit.moveDown()
    )


    fun depositMoveWall(): Action {
        return ParallelAction(
            deposit.moveDown(),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
            depositArm.moveUp(), // up to intake
            depositClaw.open(),
        )
    }

    fun depositMoveWallTeleop() = SequentialAction(
        deposit.setTargetPosition(60.0),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
        depositArm.moveDown(), // down to intake
        depositClaw.open(),
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
    fun depositScoreChamberTele() = SequentialAction(depositScoreChamberFar())//, depositArm.moveUp())

    fun depositMoveChamberFar(): Action {
        return SequentialAction(
            depositClaw.close(),
            SleepAction(0.1),
            depositArm.setPosition(0.45),
            deposit.setTargetPosition(830.0), // 1457
        )
    }

    fun depositMoveChamberFarNoGrab() = ParallelAction(
        depositArm.setPosition(0.45), deposit.setTargetPosition(830.0)
    )

    fun depositScoreChamberFar(): Action {
        return SequentialAction(
            deposit.setTargetPosition(2500.0), // 1457
            depositArm.setPosition(0.43),

            SleepAction(0.5), // 0.75
            depositClawRelease(), // TODO USE ENCODER
            //SleepAction(0.3),
            deposit.moveDown(),


            )
    }

    fun depositScoreChamberAuto() = SequentialAction(depositScoreChamberFar(), depositArm.moveMid())

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

    fun depositScoreChamberTeleop() = depositScoreChamberTele()/*
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
            return setTargetPosition(1600.0) // prev 1200
        }

        fun moveDown(): Action {
            return setTargetPosition(0.0)
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
        fun waitForTransferRelease() = SequentialAction(
            Action { return@Action !(encoder.posDegrees < 70) }, SleepAction(0.1)
        )

        fun waitForTransferGrab() = Action { return@Action !(encoder.posDegrees > 255) }
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

    class DepositArm(val depositArm: MotorControl.DepositArm, val logger: LogTelemetry) :
        ThreeArm(depositArm) {
        override fun setPosition(position: Double): Action {
            logger.update()
            logger.write("setPositionCall", "start")
            logger.update()
            return LazyAction {
                logger.update()
                logger.write("setPositionCall", "lazyStartedBuilding")
                logger.update()
                val action = goToPosAction(
                    start = depositArm.position,
                    target = position,
                    maxVel = 4.0,
                    minAccel = -4.0,
                    maxAccel = 4.0,
                    resolution = 0.01,
                    servoArm = depositArm,
                    logger = logger
                )
                logger.update()
                logger.write("setPositionCall", "lazyDoneBuilding")
                logger.update()
                action
            }
        }

        override fun moveUp(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.upPos), "depArmMoveUp", wait = false
                ), "depArm"
            )
        }

        override fun moveDown(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.downPos), "depArmMoveDown", wait = false
                ), "depArm"
            )
        }

        override fun moveMid(): Action {
            return BetterUniqueAction(
                BetterUniqueAction(
                    setPosition(depositArm.midPos), "depArmMoveFullUp", wait = false
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
    servoArm: MotorControl.ServoArm,
    logger: LogTelemetry
): Action {
    if (target == start) return Action { false }
    logger.update()
    logger.write("goToPosAction/status", "profiling")
    logger.update()
    val currentTime = ElapsedTime()
    val dispProfile =
        profile(abs(target - start), 0.0, { maxVel }, { minAccel }, { maxAccel }, resolution)
    val profile = TimeProfile(dispProfile.baseProfile)
    logger.update()
    logger.write("goToPosAction/status", "doneProfiling")
    logger.update()
    logger.write("goToPosAction/profileTimeMs", currentTime.milliseconds())
    currentTime.reset()

    var lastPosition = start
    var lastTime = currentTime.seconds()

    val negative = target < start

    logger.write("goToPosAction/start", start)
    logger.write("goToPosAction/target", target)
    logger.write("goToPosAction/maxVel", maxVel)
    logger.write("goToPosAction/minAccel", minAccel)
    logger.write("goToPosAction/maxAccel", maxAccel)
    logger.write("goToPosAction/resolution", resolution)
    logger.write("goToPosAction/negative", negative)

    logger.update()
    logger.write("goToPosAction/status", "aboutToMakeAction")
    val action = SequentialAction( // init loop
        InstantAction {
            currentTime.reset()
            lastTime = currentTime.seconds()
            logger.write("goToPosAction/status", "actionInitRunning")
        }, // init
        { // loop
            logger.update()
            logger.write("goToPosAction/status", "actionLoopStart")
            val time = currentTime.seconds()

            logger.update()
            logger.write("goToPosAction/status", "actionLoopProfileGet")
            val target = profile[time]
            logger.update()
            logger.write("goToPosAction/status", "actionLoopTargetPosGet")
            val targetPos = target[0]
            logger.update()
            logger.write("goToPosAction/status", "actionLoopTargetVelGet")
            val targetVel = target[1]
            logger.update()
            logger.write("goToPosAction/status", "actionLoopTargetAccelGet")
            val targetAccel = target[2]

            val output = if (negative) start - targetPos else start + targetPos

            logger.write("goToPosAction/lastPosition", lastPosition)
            logger.write("goToPosAction/lastTime", lastTime)
            logger.write("goToPosAction/time", time)
            logger.write("goToPosAction/targetVel", targetVel)
            logger.write("goToPosAction/targetAccel", targetAccel)
            logger.write("goToPosAction/output", output)

            logger.update()
            logger.write("goToPosAction/status", "actionLoopSetPosition")
            servoArm.position = output
            logger.update()
            logger.write("goToPosAction/status", "actionLoopDone")

            lastPosition = output
            lastTime = time
            return@SequentialAction time <= profile.duration
        })
    logger.update()
    logger.write("goToPosAction/status", "actionMade")
    return action
}
