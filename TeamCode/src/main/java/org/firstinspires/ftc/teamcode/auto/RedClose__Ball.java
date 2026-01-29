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

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose(-58, 39.5, Math.toRadians(0)));
        bot.setTelemetry(telemetry);
        ActionList a = new ActionList(bot);

        waitForStart();

        Action driveAction = bot.drive.actionBuilder(new Pose2d(-58, 39.5, Math.toRadians(0)))
                .afterTime(0, a.combine(a.setShooterVelocity(2750), a.openHardStop(), a.setTurretPos(450), a.setHood(0.74)))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-27, 22), Math.toRadians(40))
                .afterTime(0, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .afterTime(0.75, a.combine(a.closeHardStop(), a.setTurretPos(440)))
                .waitSeconds(0.75)


                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(16, 60, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                .afterTime(0, a.openHardStop())
                .afterTime(0.35, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .afterTime(1, a.combine(a.closeHardStop()))
                .waitSeconds(1)


                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(13, 66, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                .afterTime(0, a.openHardStop())
                .afterTime(0.35, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .afterTime(1, a.combine(a.closeHardStop()))
                .waitSeconds(1)


                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(44, 58, Math.toRadians(90)), Math.toRadians(90))
                .afterTime(0, a.combine(a.setIntakePower(0)))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                .afterTime(0, a.openHardStop())
                .afterTime(0.35, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .afterTime(1, a.closeHardStop())
                .waitSeconds(1)


                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(-12, 58, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                .afterTime(0, a.openHardStop())
                .afterTime(0.35, a.combine(a.setTransferPower(1), a.setIntakePower(1)))
                .afterTime(1, a.closeHardStop())
                .waitSeconds(1)
                .build();

        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop()));
    }
}
