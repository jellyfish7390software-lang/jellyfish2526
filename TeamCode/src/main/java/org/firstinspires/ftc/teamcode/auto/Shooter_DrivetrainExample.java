package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionList;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

//TODO: Feel free to copy and edit this class as a sample
@Disabled
@Autonomous
public class Shooter_DrivetrainExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        // A little class I made to s][tore all the actions for the shooter and transfer
        ActionList a = new ActionList(bot);

        // Put starting pose here
        Pose2d startPose = new Pose2d(0,0,0);

        // Updates MecanumDrive with new startPose
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Makes sure the PIDF loops are running. Just put it at the front of all your shooter
        // opmodes for good practice
        Robot.runScoringLoop = true;

        waitForStart();

        Action driveAction = drive.actionBuilder(startPose)
//                 Call your Roadrunner movements like this below (as of 10/22/25 roadrunner has not
//                 been tuned, so avoid using it for now)
//                .splineToConstantHeading(new Vector2d(20, 30), Math.toRadians(180))

                /* This is how you would call a simple time based drivetrain movement, each input of the
               function drive.setPowers() is the power of the motor, 1.0 going forwards at max speed.
               Return false is very important, make sure it is always there. I know the arrow and stuff
               makes no sense, but just go with it for now and I'll explain on Friday.
                 */
                .afterTime(0.0, telemetryPacket -> {
                    drive.setPowers(1.0, 1.0, 1.0, 1.0);
                    return false;
                })
                /* This is 0.99 and not 1 because Roadrunner does not like it when two .afterTime's have
                the same time value.
                 */
                .afterTime(0.99, telemetryPacket -> {
                    drive.setPowers(0, 0, 0, 0);
                    return false;
                })

                /* .afterTime has the code inside it run how ever many seconds you inputted after the last
                Roadrunner drivetrain movement (NOT the last .afterTime), so the transfer increment would
                run 2 seconds after the shooter command, not 3
                 */
                .afterTime(1.0, a.setShooterVelocity(4000))
                // Transfer encoder has 8192 ticks for one revolution, so this turns it 1/3 of the way
                .afterTime(3.0, a.incrementTransfer(8192/3))
                .build();


        // Runs the drive action you created in parallel with the PIDF loops for the shooter and transfer.
        // Without including the bot.scoringLoop(), the shooter and transfer would not update.
        Actions.runBlocking(new ParallelAction(driveAction, bot.scoringLoop()));
    }
}
