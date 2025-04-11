package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.MovingStatistics
import org.firstinspires.ftc.teamcode.helpers.SonicHeading
import org.firstinspires.ftc.teamcode.helpers.URM09
import kotlin.math.round

@TeleOp
class URM09HeadingTest: LinearOpMode(){
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = OctoQuadDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val rightSonic = URM09(hardwareMap.analogInput["depositArmEncoder"])
        val leftSonic = URM09(hardwareMap.analogInput["leftSonic"])

        val sonicHeadingGet = SonicHeading(leftSonic, rightSonic, 7.8)

        val leftWindow1k = MovingStatistics(1000)
        val leftWindow500 = MovingStatistics(500)
        val leftWindow100 = MovingStatistics(100)
        val leftWindow50 = MovingStatistics(50)

        val stats = arrayOf(leftWindow50,leftWindow100, leftWindow500, leftWindow1k)

        for (module: LynxModule in hardwareMap.getAll(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }




        waitForStart()

        while (opModeIsActive()) {

            for (module: LynxModule in hardwareMap.getAll(LynxModule::class.java)) {
                module.clearBulkCache()
            }

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
            val sonicHeading = sonicHeadingGet.getHeading(drive.pose.heading)

            val closest90 = round(drive.pose.heading.toDouble() / (Math.PI * 2)) * Math.PI / 2

            stats.forEach { it.add(leftSonic.distanceIn)}

            telemetry.addData("x", drive.pose.position.x)
            telemetry.addData("y", drive.pose.position.y)
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()))
            telemetry.addData("closest90",closest90 )
            telemetry.addData("sonic heading (deg)", Math.toDegrees(sonicHeading.toDouble()))
            telemetry.addData("leftDist", leftSonic.distanceIn)
            telemetry.addData("rightDist", rightSonic.distanceIn)

            stats.forEachIndexed { index, statistics -> telemetry.addData(index.toString(), statistics.mean) }
            telemetry.update()

            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose)

            packet.fieldOverlay().setStroke("#2b00ff")
            Drawing.drawRobot(packet.fieldOverlay(), Pose2d(drive.pose.position, sonicHeading))

            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}