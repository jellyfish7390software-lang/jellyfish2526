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
    public Bezier firstCycleIntake = new BezierBuilder()
            .addControlPoint(63.9375, 15.25)
            .addControlPoint(43.1, 10.9)
            .addControlPoint(29.4, 28.8)
            .addControlPoint(35.7, 61.7)
            .build();
    public Bezier firstCycleScore = new BezierBuilder()
            .addControlPoint(35.7, 61.7)
            .addControlPoint(34, 30.6)
            .addControlPoint(42, 12)
            .addControlPoint(63.9375, 15.25)
            .build();
    public Bezier secondCycleIntake = new BezierBuilder()
            .addControlPoint(63.9375, 15.25)
            .addControlPoint(64.5, 45.2)
            .addControlPoint(65.9, 58.6)
            .addControlPoint(56.3, 59.8)
            .build();
    public Bezier secondCycleScore = new BezierBuilder()
            .addControlPoint(56.3, 59.8)
            .addControlPoint(65.9, 58.6)
            .addControlPoint(64.5, 45.2)
            .addControlPoint(63.9375, 15.25)
            .build();
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
                .afterTime(0, new SequentialAction(a.setTurretPos(380), a.openHardStop(), a.setShooterVelocity(3600)))
                .waitSeconds(5)
                .afterTime(0, a.combine(a.setIntakePower(1), a.setTransferPower(1)))
                .waitSeconds(1.5)
                .afterTime(0, a.closeHardStop())


                .splineToConstantHeading(new Vector2d(36, 64), Math.toRadians(90))
                .waitSeconds(0.5)
                .afterTime(0, a.combine(a.setTransferPower(0), a.setIntakePower(0)))
                .splineToConstantHeading(new Vector2d(72 - 8.0625, 24 - 8.75), Math.toRadians(270))
                .afterTime(0, a.openHardStop())
                .waitSeconds(0.5)
                .afterTime(0, a.combine(a.setTransferPower(1), a.setIntakePower(1)))



                .build();
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop()));
    }
}
