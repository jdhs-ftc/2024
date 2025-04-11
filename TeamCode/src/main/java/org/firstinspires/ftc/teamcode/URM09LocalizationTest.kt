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
import java.lang.Math.toDegrees
import kotlin.math.abs

@TeleOp
class URM09LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = OctoQuadDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val leftSonic = URM09(hardwareMap.analogInput.get("leftSonic"))
        val leftSonicDist = SonicDistance(leftSonic, Pose2d(-4.5, -8.5, Math.toRadians(180.0)))

        val rightSonic = URM09(hardwareMap.analogInput.get("depositArmEncoder"))
        val rightSonicDist = SonicDistance(rightSonic, Pose2d(7.0, -3.0, Math.toRadians(-90.0)))

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
            val leftDist = leftSonic.distanceIn
            val rightDist = rightSonic.distanceIn
            val speed = drive.updatePoseEstimate()

            var pose = drive.pose
            val leftSonicOffset = leftSonicDist.getOffset(leftDist, pose.heading.toDouble())

            val leftSonicPos = leftSonicDist.getPosition(leftDist, pose)


            //if (leftSonicPos.minus(drive.pose.position).norm() < 24) {
            pose = Pose2d(leftSonicPos, drive.pose.heading)

            val rightSonicOffset =
                rightSonicDist.getOffset(rightDist, pose.heading.toDouble())
            val rightSonicPos = rightSonicDist.getPosition(rightDist, pose)
            //}
            //if (rightSonicPos.minus(drive.pose.position).norm() < 24) {
                pose = Pose2d(rightSonicPos, drive.pose.heading)
            //}

            if (speed.linearVel.norm() < 1.0 && toDegrees(speed.angVel.toDouble()) < 1.0) {
                drive.pose = pose
                drive.updatePoseEstimate()
            }


            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()))
            telemetry.addData(
                "rounded 90",
                abs(toDegrees(drive.pose.heading.toDouble()) % 90.0)
            )
            telemetry.addData("rounded 90 bool", abs(toDegrees(drive.pose.heading.toDouble()) % 90.0) < 10.0)
            telemetry.addData("left sonic x", leftSonicPos.x)
            telemetry.addData("left sonic y", leftSonicPos.y)
            telemetry.addData("left offset x", leftSonicOffset.x)
            telemetry.addData("left offset y", leftSonicOffset.y)
            telemetry.addData("right sonic x", rightSonicPos.x)
            telemetry.addData("right sonic y", rightSonicPos.y)
            telemetry.addData("right offset x", rightSonicOffset.x)
            telemetry.addData("right offset y", rightSonicOffset.y)
            telemetry.update()

            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose)

            packet.fieldOverlay().setStroke("#FF002b")
            Drawing.drawRobot(packet.fieldOverlay(), Pose2d(leftSonicPos,drive.pose.heading))
            packet.fieldOverlay().setStroke("#2bFF00")
            Drawing.drawRobot(packet.fieldOverlay(), Pose2d(rightSonicPos,drive.pose.heading))


            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}
