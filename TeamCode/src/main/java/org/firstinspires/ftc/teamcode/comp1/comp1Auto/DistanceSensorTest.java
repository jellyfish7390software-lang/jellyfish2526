package org.firstinspires.ftc.teamcode.comp1.comp1Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Config
@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setShooterVelocity(0);
        bot.intakePower(0);
        
        waitForStart();

        Robot.runCheckLoop = true;
        Robot.runScoringLoop = true;
        //TODO: Re-enable this to check if the auto-intaking works, make sure to comment out the while
        //TODO: loop after you do so
//        Actions.runBlocking(new ParallelAction(bot.scoringLoop(), bot.checkTransfer(), bot.driveAction(gamepad1), new LoopAction(() -> {
//            if (gamepad1.aWasPressed()) {
//                bot.setShooterVelocity(3400);
//                Robot.runCheckLoop = false;
//                Actions.runBlocking(bot.sleepWithPIDTeleop(3, gamepad1));
//                Actions.runBlocking(bot.shootFull());
//                bot.setShooterVelocity(0);
//            }
//            telemetry.addData("IntakeDistance", bot.intakeDistance.getDistance(DistanceUnit.MM));
//            telemetry.addData("BallCount", Robot.ballCount);
//            telemetry.addData("ShouldTurn", Robot.ShouldTurn);
//            telemetry.update();
//        })));

        //TODO: Same for this
//        Actions.runBlocking(new ParallelAction(bot.scoringLoop(), new SequentialAction(new SleepAction(3), bot.shootFull())));
        while (opModeIsActive() && !isStopRequested()) {

            // TODO: Only run this after re-enabling the hardwareMap.get in Robot
            telemetry.addData("IntakeDistance", bot.intakeDistance.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance", bot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
