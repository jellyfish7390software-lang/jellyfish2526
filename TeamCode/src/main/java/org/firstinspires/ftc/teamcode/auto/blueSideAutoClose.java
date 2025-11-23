package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionList;
import org.firstinspires.ftc.teamcode.LoopAction;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Vector;

//TODO: Feel free to copy and edit this class as a sample
@Autonomous(preselectTeleOp = "TeleopV1")
public class blueSideAutoClose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        // A little class I made to store all the actions for the shooter and transfer
        ActionList a = new ActionList(bot);

        // Put starting pose here
        Pose2d startPose = new Pose2d(-56,-56, Math.toRadians(216));

        // Updates MecanumDrive with new startPose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Makes sure the PIDF loops are running. Just put it at the front of all your shooter
        // opmodes for good practice
        Robot.runScoringLoop = true;

        waitForStart();

        Action driveAction = drive.actionBuilder(startPose)
                .afterTime(0.0, a.setShooterVelocity(Robot.closeRPM - 156))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-32, -32), Math.toRadians(218))

                .stopAndAdd(bot.shootFullAuto(telemetry))
                .waitSeconds(0.15)

                //TODO: First Cycle

                .afterTime(0.5, a.setShooterVelocity(0))
                .afterTime(0.51, a.startCheckLoop())
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d (-20, -42), Math.toRadians(262))
                .strafeToConstantHeading(new Vector2d(-24, -82))
                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setShooterVelocity(Robot.closeRPM - 150)))
//                .strafeToSplineHeading(new Vector2d(-18.5, 75), Math.toRadians(200))

                .strafeToLinearHeading(new Vector2d(-43, -30), Math.toRadians(223.5))

                .stopAndAdd(bot.shootFullAuto(telemetry))
                .waitSeconds(0.15)

                //TODO: Second Cycle

                .afterTime(0.5, a.setShooterVelocity(0))
                .afterTime(0.51, a.startCheckLoop())

                .strafeToLinearHeading(new Vector2d(-4, -46), Math.toRadians(264))
                .strafeToLinearHeading(new Vector2d(-4, -92), Math.toRadians(264))
                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setShooterVelocity(Robot.closeRPM - 132)))

                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(10, -48), Math.toRadians(225), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-50, 70))
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(218), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-50, 70))

                .stopAndAdd(bot.shootFullAuto(telemetry))
                .waitSeconds(0.35)

                //TODO: Third Cycle

//                .afterTime(0.5, a.setShooterVelocity(0))
//                .afterTime(0.51, a.startCheckLoop())
//
//                .strafeToLinearHeading(new Vector2d(25, -46), Math.toRadians(262))
//                .strafeToLinearHeading(new Vector2d(25, -90), Math.toRadians(262))
//                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setShooterVelocity(Robot.closeRPM - 157)))
//
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(35, -48), Math.toRadians(270), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-50, 70))
//                .strafeToSplineHeading(new Vector2d(-33, -33), Math.toRadians(220), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-50, 70))
//
//                .stopAndAdd(bot.shootFullAuto(telemetry))
//
//                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(0, -50), Math.toRadians(270))

                .build();


        // Runs the drive action you created in parallel with the PIDF loops for the shooter and transfer.
        // Without including the bot.scoringLoop(), the shooter and transfer would not update.
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop(), bot.autoCheckTransfer(), new LoopAction(() -> {
            telemetry.addData("Vel", bot.getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(bot.getRpm() - Robot.targetVel) <30);
            telemetry.update();
        })));
    }
}
