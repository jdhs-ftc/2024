package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.TranslationalVelConstraint
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.helpers.PoseStorage
import org.firstinspires.ftc.teamcode.helpers.RaceParallelAction
import org.firstinspires.ftc.teamcode.motor.MotorActions
import org.firstinspires.ftc.teamcode.motor.MotorControl
import java.lang.Math.toRadians

@Suppress("unused")
//@Autonomous(name = "Auto Sample", group = "Auto", preselectTeleOp = "Teleop Field Centric")
@Autonomous
class AutoSample : LinearOpMode() {
    lateinit var motorControl: MotorControl
    lateinit var motorActions: MotorActions
    // todo don't slam into wall
    // some weird stuff happens sometimes, is wall slam dcing??
    val openingTransferDelay: Action
        get() = motorActions.depositEncoder.waitForTransferRelease()


    override fun runOpMode() {
        val beginPose = Pose2d(-17.375, -63.0, toRadians(90.0))
        val team = PoseStorage.Team.RED

        val drive = PinpointDrive(hardwareMap, beginPose)
        motorControl = MotorControl(hardwareMap)
        motorActions = MotorActions(motorControl)

        val basketPos = Pose2d(-62.0, -61.0, toRadians(45.0))



        val traj = drive.actionBuilderPath(beginPose)
            .afterTime(0.1,motorActions.verticalTransferFull())
            .setTangent(toRadians(90.0))
            .splineToLinearHeading(basketPos, toRadians(225.0), TranslationalVelConstraint(30.0))
            .stopAndAdd(SequentialAction(
                SleepAction(1.0),
                motorActions.sampleToHighBasketBack()))
            .setTangent(toRadians(45.0))
            .splineToLinearHeading(
                Pose2d(-52.5, -48.5, toRadians(90.0)),
                toRadians(90.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(615.0),
                    motorActions.extendo.waitUntilFinished(),
                    motorActions.extendoCycle(SleepAction(0.4)),
                    SequentialAction(
                        motorActions.extendoArm.moveFullUp(),
                        motorActions.verticalTransferFull(),
                        motorActions.deposit.waitUntilFinished()
                    )
                )
            )
            .setTangent(toRadians(-90.0))
            .splineToLinearHeading(basketPos, toRadians(225.0))
            .stopAndAdd(
                SequentialAction(
                    SleepAction(1.0),
                    motorActions.sampleToHighBasketBack()
                )
            )
            .setTangent(toRadians(45.0))
            .splineToLinearHeading(
                Pose2d(-62.0, -48.5, toRadians(90.0)), // 47.1 -45.5 with back of robot in HP zone
                toRadians(90.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    motorActions.extendo.setTargetPosition(615.0),
                    motorActions.extendo.waitUntilFinished(),
                    //SleepAction(0.1), // 0.2
                    motorActions.extendoCycle(SleepAction(0.4)),
                    SequentialAction(
                        motorActions.extendoArm.moveFullUp(),
                        motorActions.verticalTransferFull(),
                        motorActions.deposit.waitUntilFinished()
                    )
                )
            )
            .setTangent(toRadians(-90.0))
            .splineToLinearHeading(basketPos, toRadians(225.0))
            .stopAndAdd(
                SequentialAction(
                    SleepAction(1.0),
                    motorActions.sampleToHighBasketBack()
                )
            )
            .setTangent(toRadians(45.0))
            .splineToLinearHeading(
                Pose2d(-62.0, -48.5, toRadians(115.0)), // 47.1 -45.5 with back of robot in HP zone
                toRadians(90.0)
            )
            // grab
            .stopAndAdd(
                SequentialAction(
                    //SleepAction(0.2), // wait for pass to finish
                    motorActions.extendo.setTargetPosition(675.0), // 800 // 850 // 900
                    motorActions.extendo.waitUntilFinished(),
                    motorActions.extendoClaw.close(),
                    motorActions.extendoArm.moveDown(),
                    SleepAction(0.1),
                    motorActions.extendoClaw.open(),
                    SleepAction(0.3),
                    motorActions.extendoGrabAndRaise(),
                    SequentialAction(
                        motorActions.extendoArm.moveFullUp(),
                        motorActions.verticalTransferFull(),
                        motorActions.deposit.waitUntilFinished()
                    )
                )
            )
            .setTangent(toRadians(-90.0))
            .splineToLinearHeading(basketPos, toRadians(225.0))
            .stopAndAdd(
                SequentialAction(
                    SleepAction(1.0),
                    motorActions.sampleToHighBasketBack()
                )
            )
            .setTangent(toRadians(45.0))
            .waitSeconds(5.0)
            .build()

        sleep(1000)
        motorControl.depositClaw.open()
        motorControl.extendoClaw.close()

        waitForStart()

        runBlocking(
            RaceParallelAction(
                traj,
                motorActions.update()
            )
        )

        PoseStorage.currentPose = drive.pose
        PoseStorage.poseUpdatedTime = System.currentTimeMillis()
        PoseStorage.currentTeam = team
    }
}
