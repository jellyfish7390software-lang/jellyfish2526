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
public class BlueFar__Ball extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(72 - 8.0625, -24 + 8.75, Math.toRadians(270)));
        ActionList a = new ActionList(bot);
        bot.setTelemetry(telemetry);


        bot.hood.setPosition(0.775);

        waitForStart();


        Action driveAction = bot.drive.actionBuilder(new Pose2d(72 - 8.0625, -24 + 8.75, Math.toRadians(270)))
                .afterTime(0, a.combine(a.setShooterVelocity(3600), a.openHardStop(), a.setTurretPos(-385), a.setHood(0.74)))
                .waitSeconds(6)
                .afterTime(0, a.combine(a.setTransferPower(0.9), a.setIntakePower(1)))
                .afterTime(1, a.combine(a.setTransferPower(0), a.setIntakePower(0), a.closeHardStop()))
                .waitSeconds(1)


                .afterTime(0.5, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(36, -24, Math.toRadians(270)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, -64), Math.toRadians(270))
                .afterTime(0, a.setTurretPos(-350))


                .strafeToConstantHeading(new Vector2d(44, -8))
                .afterTime(0, a.openHardStop())
                .afterTime(0.5, a.combine(a.setTransferPower(0.9), a.setIntakePower(1), a.openHardStop()))
                .waitSeconds(1.5)


                .afterTime(0, a.closeHardStop())
                .setTangent(Math.toRadians(250))
                .splineTo(new Vector2d(30, -64), Math.toRadians(180))
                .waitSeconds(0.75)
                .strafeToLinearHeading(new Vector2d(44, -8), Math.toRadians(270))
                .afterTime(0, a.openHardStop())
                .afterTime(0.5, a.combine(a.setTransferPower(0.9), a.setIntakePower(1), a.openHardStop()))
                .waitSeconds(1.5)


                .afterTime(0, a.closeHardStop())
                .setTangent(Math.toRadians(250))
                .splineTo(new Vector2d(30, -64), Math.toRadians(180))
                .waitSeconds(0.75)
                .strafeToLinearHeading(new Vector2d(44, -8), Math.toRadians(270))
                .afterTime(0, a.openHardStop())
                .afterTime(0.5, a.combine(a.setTransferPower(0.9), a.setIntakePower(1), a.openHardStop()))
                .waitSeconds(1.5)




                .build();
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop()));
    }
}
