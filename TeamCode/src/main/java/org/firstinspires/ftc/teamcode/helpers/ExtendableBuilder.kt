package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import org.slf4j.MarkerFactory
import kotlin.reflect.KVisibility
import kotlin.reflect.full.memberProperties

open class ExtendableBuilder(var builder: TrajectoryActionBuilder)  {
    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() = {
        builder = builder.endTrajectory()
        this
    }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds action [a] next.
     */
    fun stopAndAdd(a: Action): ExtendableBuilder {
        builder = builder.stopAndAdd(a)
        return this
    }

    fun stopAndAdd(f: InstantFunction) = stopAndAdd(InstantAction(f))

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double): ExtendableBuilder {
        builder = builder.stopAndAdd(SleepAction(t))
        return this
    }

    /**
     * Schedules action [a] to execute in parallel starting at a displacement [ds] after the last trajectory segment.
     * The action start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    fun afterDisp(ds: Double, a: Action) = {
        builder = builder.afterDisp(ds, a)
        this
    }

    fun afterDisp(ds: Double, f: InstantFunction) = afterDisp(ds, InstantAction(f))

    /**
     * Schedules action [a] to execute in parallel starting [dt] seconds after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(dt: Double, a: Action) = {
        builder = builder.afterTime(dt, a)
        this
    }

    fun afterTime(dt: Double, f: InstantFunction) = afterTime(dt, InstantAction(f))

    fun setTangent(r: Rotation2d) = {
        builder = builder.setTangent(r)
        this
    }

    fun setTangent(r: Double) = setTangent(Rotation2d.exp(r))

    fun setReversed(reversed: Boolean) = {
        builder = builder.setReversed(reversed)
        this
    }

    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null) = {
        builder = builder.turn(angle, turnConstraintsOverride)
        this
    }

    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null) = {
        builder = builder.turnTo(heading, turnConstraintsOverride)
        this
    }

    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.exp(heading), turnConstraintsOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToX(posX, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToXConstantHeading(posX, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToXLinearHeading(posX, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToXLinearHeading(posX, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToXSplineHeading(posX,heading,velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToXSplineHeading(posX, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToY(posY, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToYConstantHeading(posY, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToYLinearHeading(posY, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = lineToYLinearHeading(posY, Rotation2d.exp(heading), velConstraintOverride, accelConstraintOverride)

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToYSplineHeading(posY, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.lineToYSplineHeading(posY, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeTo(pos, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeToConstantHeading(pos, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeToConstantHeading(pos, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeToLinearHeading(pos, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeToSplineHeading(pos, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.strafeToSplineHeading(pos, heading, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineTo(pos, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToConstantHeading(pos, tangent, velConstraintOverride,accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToConstantHeading(pos, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToLinearHeading(pose, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToLinearHeading(pose, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToSplineHeading(pose, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = {
        builder = builder.splineToSplineHeading(pose, tangent, velConstraintOverride, accelConstraintOverride)
        this
    }

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = {
        ExtendableBuilder(builder.fresh())
    }

    fun build(): Action {
        return builder.build()
    }



}


class P2pBuilder(builder: TrajectoryActionBuilder, val p2pActionFactory: P2pActionFactory) : ExtendableBuilder(builder) {
    fun pid2point(target: Pose2d): P2pBuilder {

        val mappedTarget = builder.poseMap.map(target)
        val lastPose = builder.getPrivate("lastPose") as Pose2d
        builder = builder.stopAndAdd(
            p2pActionFactory.make(
                // evil cursed reflection hacks
                // lastPose is private for some reason…
                // get it and use it as a starting pose
            lastPose,
            mappedTarget))

        val endPoseUnmapped = target
        val endPose = mappedTarget
        val endTangent = endPose.minus(lastPose).line.angleCast()



        builder = TrajectoryActionBuilder::class.constructors.find {
            it.parameters.size == 8
            it.visibility == KVisibility.PRIVATE
        }!!.call(
            builder,
            TrajectoryBuilder(
                builder.trajectoryBuilderParams,
                endPoseUnmapped,
                builder.beginEndVel,
                builder.baseVelConstraint,
                builder.baseAccelConstraint,
                builder.poseMap,
            ),
            0,
            endPoseUnmapped,
            endPose,
            endTangent,
            ArrayList<MarkerFactory>(), // just needs to be empty
            builder.getPrivate("cont")
        )

        return this
    }
    fun PoseMap.map(pose: Pose2d) = map(Pose2dDual.constant(pose, 1)).value()

    fun TrajectoryActionBuilder.getPrivate(name: String) = this::class.memberProperties.find { it.name == name }!!.getter.call(this)

}

fun interface P2pActionFactory {
    fun make(startPose: Pose2d, targetPose: Pose2d): Action
}