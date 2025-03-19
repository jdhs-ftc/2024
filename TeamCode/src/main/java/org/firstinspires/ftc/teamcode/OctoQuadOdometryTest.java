package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuadFWv3;
import com.acmerobotics.roadrunner.ftc.octoquad.OctoQuadRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OctoQuadOdometryTest extends LinearOpMode
{
    static final float TICKS_PER_MM = 12.66f;
    static final float X_OFFSET_FROM_CENTER_MM = -97.05f;
    static final float Y_OFFSET_FROM_CENTER_MM = -156.70f;
    static final float OQ_IMU_SCALAR = 1.0323f;
    static final int OQ_PORT_X = 1;
    static final int OQ_PORT_Y = 2;

    public void runOpMode()
    {
        OctoQuadFWv3 oq = hardwareMap.get(OctoQuadRR.class, "octoquad");
        /*
        SkyStoneDriveBase base;
        base = new SkyStoneDriveBase();
        base.init(hardwareMap);

         */

        telemetry.setMsTransmissionInterval(50);

        oq.setSingleEncoderDirection(OQ_PORT_X, OctoQuadFWv3.EncoderDirection.FORWARD);
        oq.setSingleEncoderDirection(OQ_PORT_Y, OctoQuadFWv3.EncoderDirection.REVERSE);
        oq.setLocalizerPortX(OQ_PORT_X);
        oq.setLocalizerPortY(OQ_PORT_Y);
        oq.setLocalizerCountsPerMM_X(TICKS_PER_MM);
        oq.setLocalizerCountsPerMM_Y(TICKS_PER_MM);
        oq.setLocalizerTcpOffsetMM_X(X_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerTcpOffsetMM_Y(Y_OFFSET_FROM_CENTER_MM);
        oq.setLocalizerImuHeadingScalar(OQ_IMU_SCALAR);
        oq.setLocalizerVelocityIntervalMS(25);
        oq.resetLocalizerAndCalibrateIMU();

        while (opModeInInit())
        {
            telemetry.addData("Localizer status", oq.getLocalizerStatus());
            telemetry.addData("Heading Axis Detection", oq.getLocalizerHeadingAxisChoice());
            telemetry.update();
        }

        OctoQuadFWv3.LocalizerDataBlock localizer = new OctoQuadFWv3.LocalizerDataBlock();
        int[] encoders = new int[8];
        OctoQuadFWv3.EncoderDataBlock encoderDataBlock = new OctoQuadFWv3.EncoderDataBlock();

        while (opModeIsActive())
        {
            if (gamepad1.left_bumper)
            {
                oq.setLocalizerPose(500, 750, (float)(Math.PI/4.0f));
            }
            else if (gamepad1.right_bumper)
            {
                oq.setLocalizerHeading((float) Math.PI);
            }
            /*
            MecanumDrive.cartesian(base,
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);

             */

            oq.readLocalizerData(localizer);
            //oq.readLocalizerDataAndEncoderPositions(localizer, encoders);
            //oq.readLocalizerDataAndAllEncoderData(localizer, encoderDataBlock);

            telemetry.addData("Localizer status", localizer.localizerStatus);

            if (localizer.isPoseDataValid())
            {
                telemetry.addData("Heading deg", localizer.heading_rad*180/Math.PI);
                telemetry.addData("Heading dps", localizer.velHeading_radS*180/Math.PI);
                telemetry.addData("X mm", localizer.posX_mm);
                telemetry.addData("Y mm", localizer.posY_mm);
                telemetry.addData("VX mm/s", localizer.velX_mmS);
                telemetry.addData("VY mm/s", localizer.velY_mmS);
                //telemetry.addData("EncX", encoders[OQ_PORT_X]);
                //telemetry.addData("EncY", encoders[OQ_PORT_Y]);
                //telemetry.addData("EncX", encoderDataBlock.positions[OQ_PORT_X]);
                //telemetry.addData("EncY", encoderDataBlock.positions[OQ_PORT_Y]);
                //telemetry.addData("EncXV", encoderDataBlock.velocities[OQ_PORT_X]);
                //telemetry.addData("EncYV", encoderDataBlock.velocities[OQ_PORT_Y]);
            }
            else
            {
                telemetry.addLine("Data not valid");
            }

            telemetry.update();
        }
    }
}
