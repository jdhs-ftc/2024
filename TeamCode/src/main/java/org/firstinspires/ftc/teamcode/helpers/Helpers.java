package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class Helpers {
    public static Vector2d toVector2d(VectorF vectorF) {
        return new Vector2d(vectorF.get(0), vectorF.get(1));
    }
    /**
     * Converts from the SDK AprilTagPoseFtc coordinates to the SDK VectorF coordinates
     * To be honest I don't understand the difference and this will almost certainly cause a bug
     * But the tag positions are in VectorF
     * @param aprilTagPoseFtc the AprilTagPoseFtc to convert
     * @return equivalent VectorF
     */
    public static VectorF tagPoseToVectorF(AprilTagPoseFtc aprilTagPoseFtc) {
        return new VectorF(new float[] {
                (float) aprilTagPoseFtc.x,
                (float) aprilTagPoseFtc.y,
                (float) aprilTagPoseFtc.z
        });
    }
    public static Vector2d rotateVector(Vector2d vector, double rotationAmountRadians) {
        return new Vector2d(vector.x * Math.cos(rotationAmountRadians) - vector.y * Math.sin(rotationAmountRadians), vector.x * Math.sin(rotationAmountRadians) + vector.y * Math.cos(rotationAmountRadians));
    }
    public static Pose2d rotatePose(Pose2d pose, double rotationAmountRadians) {
        return new Pose2d(rotateVector(pose.position,rotationAmountRadians), pose.heading.log() + rotationAmountRadians);
    }

    public static double quarternionToHeading(Quaternion Q) {
        return Math.atan2(2.0 * (Q.z * Q.w + Q.x * Q.y) , - 1.0 + 2.0 * (Q.w * Q.w + Q.x * Q.x)) - Math.toRadians(270);
    }
}
