package org.firstinspires.ftc.teamcode.helpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.SleepAction;
import kotlin.random.Random;

import java.util.function.Consumer;
import java.util.function.Supplier;

// Released under the MIT License and the BSD-3-Clause license by j5155 (you may use it under either one)
public class ActionHelpers {
    public static class RaceParallelCommand implements Action {
        private final Action[] actions;

        public RaceParallelCommand(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            boolean finished = true;
            for (Action action : actions) finished = finished && action.run(t);
            return finished;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            for (Action action : actions) action.preview(canvas);
        }


    }
    public static class ActionWithUpdate implements Action {
        private final Action action;
        private final Runnable func;

        public ActionWithUpdate(Action action, Runnable func) {
            this.action = action;
            this.func = func;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            func.run();
            return action.run(t);
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            action.preview(canvas);
        }


    }

    public static class TimeoutAction implements Action {
        private final Action action;
        private final double timeout;
        private double startTime = Actions.now();

        public TimeoutAction(@NonNull Action action, long timeout) {
            this.action = action;
            this.timeout = timeout;
        }

        public TimeoutAction(@NonNull Action action) {
            this.action = action;
            this.timeout = 5;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            if (startTime == 0) startTime = Actions.now();
            if (Actions.now() - startTime > timeout) return false;
            return action.run(t);
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            action.preview(canvas);
        }
    }

    /// Takes two actions and a condition supplier as inputs.
    /// Runs the first action if the condition is true and runs the second action if it is false.
    ///
    /// Written by j5155, released under the MIT License and the BSD-3-Clause license (you may use it under either one)
    ///
    /// Example usage:
    /// ```java
    ///  new ConditionalAction(
    ///                 new SleepAction(1), // will be run if the conditional is true
    ///                 new SleepAction(2), // will be run if the conditional is false
    ///                 () -> Math.random() > 0.5); // lambda conditional function, returning either true or false;
    ///         // this example picks which one to run randomly
    ///```
    public static class ConditionalAction implements Action {
        private final Action trueAction;
        private final Action falseAction;
        private final Supplier<Boolean> condition;
        private Action chosenAction; // intentionally null

        public ConditionalAction(Action trueAction, Action falseAction, Supplier<Boolean> condition) {
            this.trueAction = trueAction;
            this.falseAction = falseAction;
            this.condition = condition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            if (chosenAction == null) {
                // if we haven't decided on an action to run yet
                // (so on the first run of this action)
                if (condition.get()) { // use .get() to check the value of the condition by running the input function
                    chosenAction = trueAction; // and then save the decision to the chosenAction variable
                } else {
                    chosenAction = falseAction;
                }
            }
            // then, every loop, pass through the chosen action
            return chosenAction.run(t);
        }

        // ambiguous which one to preview, so preview both
        @Override
        public void preview(@NonNull Canvas canvas) {
            trueAction.preview(canvas);
            falseAction.preview(canvas);
        }
    }

    Action choose() {
        return new ConditionalAction(
                new SleepAction(1), // will be run if the conditional is true
                new SleepAction(2), // will be run if the conditional is false
                () -> Math.random() > 0.5); // lambda conditional function, returning either true or false;
        // this example picks which one to run randomly
    }

    public static abstract class InitLoopAction implements Action {
        private boolean initialized = false;
        /**
         * Run exactly once the first time the action is run.
         */
        abstract public void init();

        /**
         * Run every loop. Init is guaranteed to have been run once before this.
         * @return whether to continue with the action; true to continue looping, false to end
         */
        abstract public boolean loop();

        @Override
        final public boolean run(@NonNull TelemetryPacket t) { // final to prevent downstream classes from overriding it
            if (!initialized) {
                init();
                initialized = true;
            }
            return loop();
        }

        // intentionally not overriding preview

    }

}
