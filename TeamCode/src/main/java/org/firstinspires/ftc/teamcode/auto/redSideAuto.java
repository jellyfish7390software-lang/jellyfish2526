package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionList;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

//TODO: Feel free to copy and edit this class as a sample
@Autonomous
public class redSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        // A little class I made to store all the actions for the shooter and transfer
        ActionList a = new ActionList(bot);

        // Put starting pose here
        Pose2d startPose = new Pose2d(-56,56,Math.toRadians(144));

        // Updates MecanumDrive with new startPose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Makes sure the PIDF loops are running. Just put it at the front of all your shooter
        // opmodes for good practice
        Robot.runScoringLoop = true;

        waitForStart();

        Action driveAction = drive.actionBuilder(startPose)
                .afterTime(0.0, a.setShooterVelocity(3300))
                .waitSeconds(0.5)
                .setReversed(false)
                .strafeTo(new Vector2d(-31, 31))

                .afterTime(0.25, a.incrementTransfer(8192/3))
                .afterTime(1.25, a.setIntakePower(0.5))
                .afterTime(2.25, a.incrementTransfer(8192/3))
                .afterTime(3.25, a.incrementTransfer(8192/3))

                .waitSeconds(3.75)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 38, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(12, 38, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(135))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, 24, Math.toRadians(135)), Math.toRadians(135))




                .splineTo(new Vector2d(-48, -24), Math.toRadians(135))    // was 225°
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-12, -38), Math.toRadians(-90))    // was 90°
                .waitSeconds(1)
                .splineTo(new Vector2d(-24, -24), Math.toRadians(-135))   // was 135°
                .waitSeconds(1)
                .splineTo(new Vector2d(12, -38), Math.toRadians(-90))     // was 90°
                .waitSeconds(1)
                .splineTo(new Vector2d(-24, -24), Math.toRadians(-135))   // was 135°
                .waitSeconds(1)
                .splineTo(new Vector2d(38, -38), Math.toRadians(-90))     // was 90°
                .waitSeconds(1)
                .splineTo(new Vector2d(-24, -24), Math.toRadians(-135))   // was 135°
                .build());

                //                 Call your Roadrunner movements like this below (as of 10/22/25 roadrunner has not
//                 been tuned, so avoid using it for now)
//                .splineToConstantHeading(new Vector2d(20, 30), Math.toRadians(180))

                /* This is how you would call a simple time based drivetrain movement, each input of the
               function drive.setPowers() is the power of the motor, 1.0 going forwards at max speed.
               Return false is very important, make sure it is always there. I know the arrow and stuff
               makes no sense, but just go with it for now and I'll explain on Friday.
                 */

                /* .afterTime has the code inside it run how ever many seconds you inputted after the last
                Roadrunner drivetrain movement (NOT the last .afterTime), so the transfer increment would
                run 2 seconds after the shooter command, not 3
                 */

                // Transfer encoder has 8192 ticks for one revolution, so this turns it 1/3 of the way


                .afterTime(30.0, telemetryPacket -> {
                    Robot.runScoringLoop = false;
                    return false;
                })
                .build();


        // Runs the drive action you created in parallel with the PIDF loops for the shooter and transfer.
        // Without including the bot.scoringLoop(), the shooter and transfer would not update.
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop(), bot.checkTransfer()));
    }
}
