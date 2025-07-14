package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

class SpecimenDeposit(
    val motorControl: MotorControl,
    val motorActions: MotorActions,
    var cyclesScored: Int = 0, // in tele 5; also this variable represents cycles with trajectories generated
    val cyclesToScore: Int = 6   ) { // in tele 10? 100? lots
    init {
        require(cyclesToScore > 0)
    }
    val cyclesToScoreTotal =  cyclesScored + cyclesToScore
    val humanPlayerLineUp = Vector2d(32.5, -62.0) // 36 -50
    val humanPlayerVec = Vector2d(32.5, -63.5) // -64.1

    val specimenDepositY = -32.0 // prev 33

    val startx = 9.0
    val offset = -1.55

    val specimenDepositX: Double
        get() = startx + (offset * cyclesScored)

    var cyclesRuntimeScored: Int = cyclesScored

    fun incrementRuntimeScoredAction() = InstantAction { cyclesRuntimeScored += 1 }

    // MUST BE LINED UP TO GRAB BEFORE RUNNING THIS, with depositMoveWall IN PLACE
    // but this will grab
    fun genTrajectory(builder: TrajectoryActionBuilder): Action {
        if (cyclesRuntimeScored != cyclesScored) { // some have already been ACTUALLY RUNTIME scored??
            cyclesScored = cyclesRuntimeScored // so start from there
        }
        // starting from grabbed from human player
        var builder = builder
            // score the first one
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.moveDown(),
                    motorActions.depositPickupWall(),
                    SleepAction(0.2),
                    motorActions.depositMoveChamber(),
                )

            )
            .setTangent(toRadians(90.0))
            .splineToConstantHeading(Vector2d(specimenDepositX, specimenDepositY), toRadians(90.0)) //back to sub
            .stopAndAdd(
                SequentialAction(
                    motorActions.depositScoreChamberTele(),
                    SleepAction(0.1),
                    motorActions.depositClaw.open(),
                    incrementRuntimeScoredAction()
                )
            )
        cyclesScored += 1
        while (cyclesScored <= cyclesToScoreTotal) {
            builder = builder.setTangent(toRadians(-90.0))
                .afterTime(0.5, motorActions.depositMoveWall())
                .splineToConstantHeading(
                    humanPlayerLineUp,
                    toRadians(270.0)
                ) // go to line up point
                // vision align should go here
                .splineToConstantHeading(
                    humanPlayerVec,
                    toRadians(270.0)
                )//, TranslationalVelConstraint(10.0)) // go to hp 35.0 -62.5
                .stopAndAdd(
                    SequentialAction(
                        motorActions.extendo.moveDown(),
                        motorActions.depositPickupWall(),
                        SleepAction(0.2),
                        motorActions.depositMoveChamber(),
                    )

                )
                .setTangent(toRadians(90.0)) // 135
                .splineToConstantHeading(
                    Vector2d(specimenDepositX, specimenDepositY),
                    toRadians(90.0)
                ) //back to sub
                .stopAndAdd(
                    SequentialAction(
                        motorActions.depositScoreChamberTele(),
                        SleepAction(0.1),
                        motorActions.depositClaw.open(),
                        incrementRuntimeScoredAction()
                    )
                )
                .setTangent(toRadians(-90.0))
            cyclesScored += 1
        }



        return builder.build()

    }

    fun genTrajectory(drive: MecanumDrive): Action = genTrajectory(drive.actionBuilderPathLowRes(Pose2d(humanPlayerVec,toRadians(90.0))))
}