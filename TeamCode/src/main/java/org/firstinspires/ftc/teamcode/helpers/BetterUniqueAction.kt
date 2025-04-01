package org.firstinspires.ftc.teamcode.helpers

import android.content.Context
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier
import com.qualcomm.robotcore.util.WebHandlerManager
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

open class BetterUniqueAction(val action: Action, val key: String = "UniqueAction") : Action by action {
    override fun run(p: TelemetryPacket): Boolean {
        if (!UniqueActionQueue.runningUniqueActions.contains(this) && UniqueActionQueue.runningUniqueActions.any { it.key == key }) (
            // this action is duplicated
            if (UniqueActionQueue.shouldQueueUniqueActions) {
                // wait for the other one to finish
                return true
            } else {
                // end immediately
                return false
            }
        ) else ( // not duplicated
            // run the real action
            if (action.run(p)) { // if the real action wants to run again
                // add this action to the running actions if it isn't there already
                if (!UniqueActionQueue.runningUniqueActions.contains(this)) (
                    UniqueActionQueue.runningUniqueActions.add(this)
                )
                return true
            } else {
                // either the real action has just finished or it was instant
                // remove is safe either way; if we were never added to the list nothing happens

                // note: it's possible, if an action is forcibly not queued like with raceaction
                // or if the opmode is stopped, that this never runs
                //  UniqueActionQueue notifications deal with opmode stopping
                UniqueActionQueue.runningUniqueActions.remove(this)
                return false
            }
        )
    }
}

object UniqueActionQueue : OpModeManagerNotifier.Notifications {

    val runningUniqueActions = ArrayList<BetterUniqueAction>()
    var shouldQueueUniqueActions = true // can be changed at runtime
    // (not sure whether this is good; allows me to make super ultra unique actions)

    // stolen from rr ftc, note from rrbrott
    // I'm tempted to use @OnCreate, but some of the hooks are unreliable and @WebHandlerRegistrar
    // seems to just work.
    @WebHandlerRegistrar
    @JvmStatic
    fun registerRoutes(context: Context, manager: WebHandlerManager) {
        OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().activity)
            .registerListener(this)
    }


    override fun onOpModePreInit(p0: OpMode) {
        runningUniqueActions.clear()
        shouldQueueUniqueActions = true
    }

    override fun onOpModePreStart(p0: OpMode) {

    }

    override fun onOpModePostStop(p0: OpMode) {

    }
}