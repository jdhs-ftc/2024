package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.round
import kotlin.math.sin

private const val MAX_RANGE = 520 // cm
class URM09(val channel: AnalogInput) {
    val distanceCm: Double
        get() = channel.voltage * MAX_RANGE / channel.maxVoltage // reads hardware!
    fun getDistance(unit: DistanceUnit) = unit.fromCm(distanceCm)

    val distanceIn: Double
        get() = getDistance(DistanceUnit.INCH)
}

class SonicHeading(val leftSonic: URM09, val rightSonic: URM09,
                     val distBetweenSensorsIn: Double) {
    fun getHeading(currentHeading: Rotation2d): Rotation2d = Rotation2d.fromDouble(getHeading(currentHeading.toDouble()))

    fun getHeading(currentHeadingRad: Double): Double { // might work???
        val closest90 = round(currentHeadingRad / (Math.PI * 2)) * Math.PI / 2
        val offset = atan((rightSonic.distanceIn - leftSonic.distanceIn)/distBetweenSensorsIn)
        return closest90 + offset
    }
}

class SonicDistance(val sonic: URM09, val sensorOffset: Pose2d = Pose2d(0.0,0.0,0.0)) {
    // does this defaulting even make any sense?
    fun getPosition(distance: Double = sonic.distanceCm, pose: Pose2d): Vector2d {
        val heading = (pose.heading + sensorOffset.heading.toDouble()).toDouble()
        if (abs(toDegrees(heading) % 90.0) < 10.0) { // screw working while heading is off
            val wall = Wall.getWall(heading)
            return getPosition(distance, pose, wall)
        } else {
            return pose.position
        }
    }

    fun getPosition(distance: Double, pose: Pose2d, wall: Wall): Vector2d {
        val heading = (pose.heading + sensorOffset.heading.toDouble()).toDouble()

        val rotatedSensorOffset = pose.heading * sensorOffset.position

        val wallOffset = getOffset(distance, heading)

        // good chance this math is wrong
        return when (wall.direction) {
            WallDirection.X -> Vector2d(
                rotatedSensorOffset.x + wall.distance - wallOffset.y,
                pose.position.y
            )
            WallDirection.Y -> Vector2d(
                pose.position.x,
                rotatedSensorOffset.y + wall.distance - wallOffset.x
            )
        }
    }

    // distance and heading OF A SENSOR (possibly 180 from bot heading)
    // output field centric
    fun getOffset(distance: Double, heading: Double): Vector2d {
        var x = distance * sin(heading)
        val y = distance * cos(heading)
        return Vector2d(x, y)
    }
}


enum class WallDirection {
    X,
    Y
}
// direction is the line that the wall is on (PERPENDICULAR TO)
// dist is the distance in incehs along the direction
enum class Wall(val direction: WallDirection, val distance: Double) {
    RED(WallDirection.Y, -72.0), // negative y, 270 deg
    BLUE(WallDirection.Y, 72.0), // positive y, 90 deg
    AUDIENCE(WallDirection.X, 72.0), // positive x, 0 deg
    REF(WallDirection.X, -72.0); // negative x ,180 deg

    companion object {
        // not correct around edges/corners
        // but idk that I really care
        fun getWall(heading: Double): Wall {
            val closest90 = (round(heading / toRadians(90.0)) * toRadians(90.0)) % toRadians(360.0)

            return when (closest90) {
                toRadians(0.0) -> AUDIENCE
                toRadians(90.0) -> BLUE
                toRadians(180.0) -> REF
                toRadians(-180.0) -> REF
                toRadians(-90.0) -> RED
                else -> throw RuntimeException("Rounding didn't work?? Closest90 is $closest90")
            }
        }
    }
}




// the rest of this bad
open class Line2d(val a: Vector2d, val b: Vector2d) {
    val slope: Double
        get() = (b.y - a.y) / (b.x - a.x)
    val yIntercept: Double
        get() = a.y - slope * a.x

    fun get(x: Double) = slope * x + yIntercept

    fun intersect(other: Line2d): Vector2d {
        val a1 = this.a.y-this.b.y
        val b1 = this.a.x-this.b.x
        val c1 = a1 * this.a.x +b1 * this.a.y

        //val d1 = Vector2d(this.a.y-this.b.y, this.a.x-this.b.x) dot this.a

        val a2 = other.b.y - other.a.y
        val b2 = other.b.x - other.a.x
        val c2 = a2 * other.a.x +b2 * other.a.y

       // val d2 = Vector2d(other.b.y-other.a.y, other.b.x-other.a.x) dot other.a

        val delta = a1 * b2 - a2 * b1
        return Vector2d ((b2 * c1 - b1 * c2) / delta, (a1 * c2-a2 * c1) / delta)
    }
}

class Segment2d(a: Vector2d, b: Vector2d): Line2d(a, b) {
    val length: Double
        get() = (a - b).norm()


}