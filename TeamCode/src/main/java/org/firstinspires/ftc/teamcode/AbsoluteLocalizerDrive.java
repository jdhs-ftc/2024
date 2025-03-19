package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LocalizationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.PoseMessage;

abstract public class AbsoluteLocalizerDrive extends MecanumDrive {
    public LocalizationSensor localSensor;
    Pose2d lastPose = pose;
    public AbsoluteLocalizerDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        localSensor = setupLocalization(hardwareMap);
    }

    abstract public LocalizationSensor setupLocalization(HardwareMap hardwareMap);

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPose != pose) {
            // Something else is modifying our pose (likely for relocalization),
            // so we override the sensor's pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            localSensor.writePose(pose);
        }
        localSensor.updatePoseVel();
        pose = localSensor.getCachedPose();
        lastPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return localSensor.getCachedVel();
    }
}
