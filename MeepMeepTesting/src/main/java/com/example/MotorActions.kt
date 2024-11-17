package com.example

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction

class MotorActions() {
    fun log(message: String): Action {
        return InstantAction { println(message) }
    }

    // presets
    fun extendoClawGround(): Action {
        return SequentialAction(
//            extendoClaw.open(), // open claw
//            extendoArm.moveDown(), // move to ground
        )
    }

    fun extendoGrabAndRaise(): Action {
        return SequentialAction(
//            extendoClaw.close(), // close claw
            SleepAction(0.5),
//            extendoArm.moveUp() // move claw to "clears ground bar" pos
        )
    }

    fun extendoCycle(between: Action = SleepAction(0.5)): Action {
        return SequentialAction(
            extendoClawGround(),
            between, // default wait 0.5
            extendoGrabAndRaise()
        )
    }

    fun depositMoveWall(): Action {
        return SequentialAction(
//            deposit.moveDown(),//deposit.setTargetPosition(150.0), // previously 116 // Tuned as of 10/24
//            extendo.moveDown(),
//            depositArm.moveDown(), // down to intake
//            depositClaw.open(),
//            Action {!(motorControl.extendo.position < 300) }, // wait for extendo to be retracted, todo tune
//            extendoArm.moveDump()
        )
    }

    fun depositPickupWall(): Action {
        return SequentialAction(
//        extendoClaw.open(),
//        depositClaw.close(),
        SleepAction(0.5), // TODO tune
//        deposit.setTargetPosition(250.0)
        )
    }

    fun depositMoveChamber(): Action {
        return SequentialAction(
//            depositArm.moveUp(),
//            deposit.setTargetPosition(1000.0), // TODO TUNEME
        )
    }

    fun depositScoreChamber(): Action {
        return SequentialAction(
//            deposit.setTargetPosition(1200.0),
        )
    }
    fun depositClawRelease(): Action {
        return SequentialAction()
    }
}
