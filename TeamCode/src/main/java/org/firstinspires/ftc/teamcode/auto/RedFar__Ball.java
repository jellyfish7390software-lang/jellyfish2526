package org.firstinspires.ftc.teamcode.auto;

import static java.util.Arrays.asList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.actions.ActionList;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierBuilder;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;

@Autonomous
@Config
public class RedFar__Ball extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(72 - 8.0625, 24 - 8.75, Math.toRadians(90)));
        ActionList a = new ActionList(bot);

        waitForStart();

        /*Actions.runBlocking(new RaceAction(new SequentialAction(
                a.setTurretPos(385),
                a.setShooterVelocity(3600),
                a.openHardStop(),
                bot.sleepWithPID(5),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(1),
//                a.setTransferPower(0),
//                a.setIntakePower(0),
                a.closeHardStop(),


                bot.followPathConstantHeading(firstCycleIntake, Math.toRadians(90)),
                bot.sleepWithPID(0.5),
                bot.followPathConstantHeading(firstCycleScore, Math.toRadians(90)),
                a.stopDt(),
                a.openHardStop(),
                bot.sleepWithPID(0.75),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(0.75),
                a.closeHardStop(),
                a.setTurretPos(400),


                bot.followPath(secondCycleIntake),
                bot.sleepWithPID(0.5),
                bot.followPathConstantHeading(secondCycleScore, Math.toRadians(90)),
                a.stopDt(),
                a.openHardStop(),
                bot.sleepWithPID(0.75),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(0.75),
                a.closeHardStop(),


                bot.followPath(secondCycleIntake),
                bot.sleepWithPID(0.5),
                bot.followPathConstantHeading(secondCycleScore, Math.toRadians(90)),
                a.stopDt(),
                a.openHardStop(),
                bot.sleepWithPID(0.75),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(0.75),
                a.closeHardStop()
                        */

        Action driveAction = bot.drive.actionBuilder(new Pose2d(72 - 8.0625, 24 - 8.75, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(36, 24, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, 64), Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(56, 10))
                .waitSeconds(1)
                .setTangent(Math.toRadians(110))
                .splineTo(new Vector2d(50, 64), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(110))
                .splineTo(new Vector2d(50, 64), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(90))




                .build();
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop()));
    }
}
