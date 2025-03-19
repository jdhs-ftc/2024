package org.firstinspires.ftc.teamcode;



import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LocalizationSensor;
import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuadFWv3;
import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuadRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Experimental extension of MecanumDrive that uses the OctoQuad sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 */
@Config
public class OctoQuadDrive extends AbsoluteLocalizerDrive {
    public static class Params {
        /*
        Set this to the name that your Octoquad is configured as in your hardware config.
         */
        public String octoquadDeviceName = "octoquad";

        /*
        Set these to the numbers of the ports that the X and Y odometry are plugged into on the OctoQuad.
         */
        public int odometryPortX = 1;

        public int odometryPortY = 2;

        /*
        The OctoQuad IMU needs to be tuned before use to ensure the output heading is accurate.
        Run AngularScalarTuner and follow the instructions to get this value.
         */
        public double angularScalar = 1.0415;

        /*
        Set the odometry pod positions relative to the center of the robot.
        The X pod offset refers to how far sideways from the center the X (forward) odometry pod is.
        Left of the center is a positive number, right of the center is a negative number.
        The Y pod offset refers to how far forwards from the center the Y (strafe) odometry pod is:
        forward of the center is a positive number, backwards is a negative number.
        See the OctoQuad quickstart guide for a better explanation.
         */
        // These are tuned for 3110-0002-0001 Product Insight #1
        public double xOffset = -5.24373777; // inches
        public double yOffset = -3.412719295440588; // inches

        /*
        Set the encoder resolution of your odometry pods in ticks per millimeter.

        This is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; for example, the value for the Gobilda 4 Bar Odometry Pod is 19.89436789.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        public double encoderResolution = 19.89436789; // ticks / mm

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public OctoQuadFWv3.EncoderDirection xDirection = OctoQuadFWv3.EncoderDirection.FORWARD;
        public OctoQuadFWv3.EncoderDirection yDirection = OctoQuadFWv3.EncoderDirection.REVERSE;

        /*
        Use the OctoQuad IMU for tuning
        If true, overrides any IMU setting in MecanumDrive and uses exclusively OctoQuad for tuning
        You can also use the OctoQuad directly in MecanumDrive if this doesn't work for some reason;
         replace "imu" with "octoquad" or whatever your octoquad is called in config.
         Note: OctoQuad IMU is always used for base localization
         */
        public boolean useOctoQuadIMUForTuning = true;
    }
    public static Params PARAMS = new Params();

    public OctoQuadDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
    }

    @Override
    public LocalizationSensor setupLocalization(HardwareMap hardwareMap) {
        FlightRecorder.write("OCTOQUAD_PARAMS",PARAMS);
        OctoQuadRR octoquad = hardwareMap.get(OctoQuadRR.class,PARAMS.octoquadDeviceName);

        if (PARAMS.useOctoQuadIMUForTuning) {
            lazyImu = new LazyImu(hardwareMap, PARAMS.octoquadDeviceName, new RevHubOrientationOnRobot(zyxOrientation(0, 0, 0)));
        }

        octoquad.setLocalizerPortX(PARAMS.odometryPortX);
        octoquad.setLocalizerPortY(PARAMS.odometryPortY);

        octoquad.setSingleEncoderDirection(PARAMS.odometryPortX,PARAMS.xDirection);
        octoquad.setSingleEncoderDirection(PARAMS.odometryPortY,PARAMS.yDirection);

        // RR localizer note: don't love this conversion (change driver?)
        octoquad.setLocalizerTcpOffsetMM_X((float)DistanceUnit.MM.fromInches(PARAMS.xOffset));
        octoquad.setLocalizerTcpOffsetMM_Y((float)DistanceUnit.MM.fromInches(PARAMS.yOffset));


        octoquad.setLocalizerCountsPerMM_X((float) PARAMS.encoderResolution);
        octoquad.setLocalizerCountsPerMM_Y((float) PARAMS.encoderResolution);

        octoquad.setLocalizerImuHeadingScalar((float) PARAMS.angularScalar);

        octoquad.setLocalizerVelocityIntervalMS(25);


        /*
        Reset the localization and calibrate the IMU.
         */
        octoquad.baseInitialize();

        octoquad.writePose(pose);

        return octoquad;
    }

}
