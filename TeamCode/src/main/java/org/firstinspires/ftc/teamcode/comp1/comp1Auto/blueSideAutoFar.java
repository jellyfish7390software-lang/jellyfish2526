package org.firstinspires.ftc.teamcode.comp1.comp1Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.ActionList;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.comp1.roadrunner.MecanumDrive;

//TODO: Feel free to copy and edit this class as a sample
@Autonomous(preselectTeleOp = "TeleopV1")
public class blueSideAutoFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        // A little class I made to store all the actions for the shooter and transfer
        ActionList a = new ActionList(bot);

        // Put starting pose here
        Pose2d startPose = new Pose2d(64,-18,Math.toRadians(180));

        // Updates MecanumDrive with new startPose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Makes sure the PIDF loops are running. Just put it at the front of all your shooter
        // opmodes for good practice
        Robot.runScoringLoop = true;

        waitForStart();

        Action driveAction = drive.actionBuilder(startPose)
                .afterTime(0.0, a.setShooterVelocity(Robot.farRPM - 200))
//              .waitSeconds(0.5)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(48, -12), Math.toRadians(207))

//              .waitSeconds(0.25)
                .stopAndAdd(bot.shootFullAutoFar(telemetry))
                .waitSeconds(0.25)

                .afterTime(0.5, a.setShooterVelocity(0))
                .afterTime(0.51, a.startCheckLoop())

                .strafeToLinearHeading(new Vector2d(23, -22), Math.toRadians(268))
                .strafeToLinearHeading(new Vector2d(23, -72), Math.toRadians(268))
                .waitSeconds(0.5)

                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setShooterVelocity(Robot.farRPM - 150)))

                .strafeToLinearHeading(new Vector2d(48, -12), Math.toRadians(200))

                .stopAndAdd(bot.shootFullAutoFar(telemetry))
                .waitSeconds(0.25)

                .afterTime(0.5, a.setShooterVelocity(0))
                .afterTime(0.51, a.startCheckLoop())

                .strafeToLinearHeading(new Vector2d(48, -22), Math.toRadians(279))
                .splineToLinearHeading(new Pose2d(56, -62, Math.toRadians(270)), Math.toRadians(270))
                .turnTo(Math.toRadians(315))
                .waitSeconds(0.5)

                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setShooterVelocity(Robot.farRPM - 150)))

                .strafeToLinearHeading(new Vector2d(48, -8), Math.toRadians(200))

                .stopAndAdd(bot.shootFullAutoFar(telemetry))
                .waitSeconds(0.25)

                .afterTime(0.5, a.setShooterVelocity(0))
                .afterTime(0.51, a.startCheckLoop())

                .strafeToLinearHeading(new Vector2d(48, -22), Math.toRadians(279))
                .splineToLinearHeading(new Pose2d(56, -62, Math.toRadians(270)), Math.toRadians(270))
                .waitSeconds(0.5)

                .afterTime(1, new SequentialAction(a.stopCheckLoop()))

//                .strafeToLinearHeading(new Vector2d(56, 8), Math.toRadians(156.5))
//
//                .stopAndAdd(bot.shootFullAutoFar(telemetry))
//                .waitSeconds(0.25)

                .strafeToLinearHeading(new Vector2d(28, -36), Math.toRadians(180))

                .build();


        // Runs the drive action you created in parallel with the PIDF loops for the shooter and transfer.
        // Without including the bot.scoringLoop(), the shooter and transfer would not update.
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop(), bot.autoCheckTransfer()));
    }
}
