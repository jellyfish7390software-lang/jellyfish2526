package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LoopAction;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TeleopV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setShooterVelocity(0);
        bot.intakePower(0);
        bot.setGamepads(gamepad1, gamepad2);

        waitForStart();

        Robot.runCheckLoop = false;
        Robot.runScoringLoop = true;

        Actions.runBlocking(new ParallelAction(bot.scoringLoop(), bot.checkTransfer(), bot.driveAction(gamepad1), new LoopAction(() -> {
            if (gamepad1.aWasPressed()) {
                bot.setShooterVelocity(3400);
                Robot.runCheckLoop = true;
                Actions.runBlocking(bot.sleepWithPIDTeleop(3, gamepad1));
                Actions.runBlocking(bot.shootFull());
                bot.setShooterVelocity(0);
            }
            if (gamepad1.right_trigger > 0) {
                bot.intakePower(1);
                Robot.runCheckLoop = true;
            }
            else {
                bot.intakePower(0);
                Robot.runCheckLoop = false;
            }

            telemetry.addData("runCheckLoop", Robot.runCheckLoop);
            telemetry.addData("BallDist", Robot.ballDist);
            telemetry.addData("timer", bot.timer.seconds());
            telemetry.addData("IntakeDistance", bot.intakeDistance.getDistance(DistanceUnit.MM));
            telemetry.addData("BallCount", Robot.ballCount);
            telemetry.addData("ShouldTurn", Robot.ShouldTurn);
            telemetry.update();
        })));

    }
}
