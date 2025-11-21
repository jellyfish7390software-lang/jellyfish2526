package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionList;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

//TODO: Feel free to copy and edit this class as a sample
@Autonomous
public class redSideAutoFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        // A little class I made to store all the actions for the shooter and transfer
        ActionList a = new ActionList(bot);

        // Put starting pose here
        Pose2d startPose = new Pose2d(60,24,Math.toRadians(180));

        // Updates MecanumDrive with new startPose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Makes sure the PIDF loops are running. Just put it at the front of all your shooter
        // opmodes for good practice
        Robot.runScoringLoop = true;

        waitForStart();

        Action driveAction = drive.actionBuilder(startPose)
                .afterTime(0.0, a.setShooterVelocity(Robot.farRPM - 2400))
//              .waitSeconds(0.5)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(197))

//              .waitSeconds(0.25)
                .stopAndAdd(bot.shootFullAuto(telemetry))
                .waitSeconds(0.25)

//                .afterTime(0.5, a.setShooterVelocity(0))
//                .afterTime(0.51, a.startCheckLoop())
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d (-11.5, 22), Math.toRadians(90))
//                .waitSeconds(0.25)
//                .strafeToConstantHeading(new Vector2d(-11.5, 60))
////                .waitSeconds(0.5)
//
//                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setIntakePower(0)))
//
//                .afterTime(0.01, a.setShooterVelocity(Robot.closeRPM - 100))
//                .splineToSplineHeading(new Pose2d(-34, 34, Math.toRadians(140)), Math.toRadians(225))
//                .waitSeconds(0.25)
//                .stopAndAdd(bot.shootFullAuto(telemetry))
//                .waitSeconds(0.25)
//
//                .afterTime(0.5, a.setShooterVelocity(0))
//                .afterTime(0.51, a.startCheckLoop())
//
//                .strafeToLinearHeading(new Vector2d(10, 22), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(10, 68), Math.toRadians(90))
//                .waitSeconds(1)
//                .afterTime(1, new SequentialAction(a.stopCheckLoop(), a.setIntakePower(0)))
//
//                .afterTime(0.01, a.setShooterVelocity(Robot.closeRPM - 100))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-35, 30, Math.toRadians(140)), Math.toRadians(225))
//                .waitSeconds(0.25)
//                .setReversed(false)
//                .stopAndAdd(bot.shootFullAuto(telemetry))
//                .waitSeconds(0.5)
//

//
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(-24, 24), Math.toRadians(135))
//                .waitSeconds(3)
//                .afterTime(0, () -> bot.shootFull())
//
//
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(12, 38, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(3)
//                .afterTime(0, () -> bot.shootFull())
//
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(135))
//                .waitSeconds(3)
//                .afterTime(0, () -> bot.shootFull())
//
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(3)
//                .afterTime(0, () -> bot.shootFull())
//
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(135))
//                //                 Call your Roadrunner movements like this below (as of 10/22/25 roadrunner has not
////                 been tuned, so avoid using it for now)
////                .splineToConstantHeading(new Vector2d(20, 30), Math.toRadians(180))
//
//                /* This is how you would call a simple time based drivetrain movement, each input of the
//               function drive.setPowers() is the power of the motor, 1.0 going forwards at max speed.
//               Return false is very important, make sure it is always there. I know the arrow and stuff
//               makes no sense, but just go with it for now and I'll explain on Friday.
//                 */
//
//                /* .afterTime has the code inside it run how ever many seconds you inputted after the last
//                Roadrunner drivetrain movement (NOT the last .afterTime), so the transfer increment would
//                run 2 seconds after the shooter command, not 3
//                 */
//
//                // Transfer encoder has 8192 ticks for one revolution, so this turns it 1/3 of the way

                .afterTime(30.0, telemetryPacket -> {
                    Robot.runScoringLoop = false;
                    return false;
                })
                .build();


        // Runs the drive action you created in parallel with the PIDF loops for the shooter and transfer.
        // Without including the bot.scoringLoop(), the shooter and transfer would not update.
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop(), bot.autoCheckTransfer()));
    }
}
