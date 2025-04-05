package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.helpers.SonicDistance
import org.firstinspires.ftc.teamcode.helpers.URM09
@TeleOp
class URM09LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = OctoQuadDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val sonic = URM09(hardwareMap.analogInput.get("depositArmEncoder"))
        val sonicDist = SonicDistance(sonic, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            drive.setDrivePowers(
                PoseVelocity2d(
                    Vector2d(
                        -gamepad1.left_stick_y.toDouble(),
                        -gamepad1.left_stick_x.toDouble()
                    ),
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.updatePoseEstimate()
            val dist = sonic.distanceIn

            val sonicOffset = sonicDist.getOffset(dist, drive.pose.heading.toDouble())

            val sonicPos = sonicDist.getPosition(dist, drive.pose)
            if (sonicPos.minus(drive.pose.position).norm() < 24) {
                drive.pose = Pose2d(sonicPos, drive.pose.heading)
            }

            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()))
            telemetry.addData("sonic x", sonicPos.x)
            telemetry.addData("sonic y", sonicPos.y)
            telemetry.addData("offset x", sonicOffset.x)
            telemetry.addData("offset y", sonicOffset.y)
            telemetry.update()

            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose)

            packet.fieldOverlay().setStroke("#2b00ff")
            Drawing.drawRobot(packet.fieldOverlay(), Pose2d(sonicPos,drive.pose.heading))
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}
