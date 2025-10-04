package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class ShootingDistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60, 10.5, Math.toRadians(135)));
        waitForStart();

        Action driveAction = drive.actionBuilder(new Pose2d(60,10.5, Math.toRadians(135)))
                .splineTo(new Vector2d(-20, 26), Math.toRadians(135))
                .build();

        Actions.runBlocking(driveAction);

    }

}
