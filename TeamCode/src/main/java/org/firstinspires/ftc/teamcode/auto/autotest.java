package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class autotest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0, 0, 0));

        waitForStart();

        Action driveAction = drive.actionBuilder(new Pose2d(0,0,0))
        .splineTo(new Vector2d(-48, 24), Math.toRadians(225))
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-12, 38), Math.toRadians(90))
                .waitSeconds(1)
                .splineTo(new Vector2d(-24,24), Math.toRadians(135))
                .waitSeconds(1)
                .splineTo(new Vector2d(12, 38), Math.toRadians(90))
                .waitSeconds(1)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(135))
                .waitSeconds(1)
                .splineTo(new Vector2d(38,38), Math.toRadians(90))
                .waitSeconds(1)
                .splineTo(new Vector2d(-24,24), Math.toRadians(135))
                .build();

        Actions.runBlocking(driveAction);

    }

}
