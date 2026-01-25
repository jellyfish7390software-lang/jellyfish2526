package org.firstinspires.ftc.teamcode.auto;

import static java.util.Arrays.asList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
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
public class RedClose__Ball extends LinearOpMode {
    public Bezier start = new BezierBuilder()
            .addControlPoint(-58, 39.5)
            .addControlPoint(-40, 29)
            .addControlPoint(-27, 22)
            .build();
    public Bezier firstCycleIntake = new BezierBuilder()
            .addControlPoint(-27, 22)
            .addControlPoint(26, 20.2)
            .addControlPoint(18.2, 49.7)
            .addControlPoint(14, 64)
            .build();
    public Bezier firstCycleScore = new BezierBuilder()
            .addControlPoint(10.4, 63.8)
            .addControlPoint(10.8,17)
            .addControlPoint(-30, 30)
            .build();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(-58, 39.5, Math.toRadians(0)));
        ActionList a = new ActionList(bot);

        waitForStart();

        Actions.runBlocking(new RaceAction(new SequentialAction(
                a.setShooterVelocity(2550),
                a.openHardStop(),
                bot.sleepWithPID(1.5),
                a.setTurretPos(450),
                bot.followPathConstantHeading(start, Math.toRadians(40)),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(1),
//                a.setTransferPower(0),
//                a.setIntakePower(0),
                a.closeHardStop(),
                bot.followPathConstantHeading(firstCycleIntake, Math.toRadians(90)),
                bot.sleepWithPID(0.5),
                a.setTurretPos(550),
                bot.followPathConstantHeading(firstCycleScore, Math.toRadians(40)),
                a.stopDt(),
                a.openHardStop(),
                bot.sleepWithPID(0.75),
                a.setTransferPower(1),
                a.setIntakePower(1),
                bot.sleepWithPID(0.75)


        ), bot.scoringLoop()));
    }
}
