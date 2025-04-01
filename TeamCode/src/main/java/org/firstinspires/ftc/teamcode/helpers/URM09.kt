package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.lang.Math.toRadians
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

class SonicDistance {

    fun getPosition(distance: Double, pose: Pose2d): Vector2d {
        val heading = pose.heading.toDouble()

        TODO()
    }

    // distance and heading OF A SENSOR (possibly 180 from bot heading)
    fun getOffset(distance: Double, heading: Double): Vector2d {
        var x = distance * sin(heading)
        val y = distance * cos(heading)

        val wrappedHeading = (heading + toRadians(45.0)) % toRadians(90.0) - toRadians(45.0) // wraps heading to -45, 45

        // check if triangle is toward the left or the right
        if (wrappedHeading < 0) { x = -x }

        return Vector2d(x, y)
    }
}


enum class WallDirection {
    X,
    Y
}
// direction is the line that the wall is on (PERPENDICULAR TO)
// dist is the distance in incehs along the direction
class Wall(val direction: WallDirection, val distance: Double)

enum class Walls(val wall: Wall) {
    RED(Wall(WallDirection.Y, -72.0)), // negative y
    BLUE(Wall(WallDirection.Y, 72.0)), // positive y
    AUDIENCE(Wall(WallDirection.X, 72.0)), // positive x
    REF(Wall(WallDirection.X, -72.0)) // negative x
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