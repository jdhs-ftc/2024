package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class RaceAction(
    initialActions: List<Action>
) : Action {
    var actions = initialActions
        private set
    var interrupting = false

    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        val remaining = actions.filter { it.run(p) }
        if (interrupting) {
            return remaining.isNotEmpty()
        } else if (actions.size != remaining.size) {
            interrupting = true
            actions = remaining.filter { it is Interruptible }
                .map { (it as Interruptible).onInterrupt() }
        }
        return true
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in actions) {
            a.preview(fieldOverlay)
        }
    }
}

interface Interruptible: Action {
    fun onInterrupt(): Action
}
