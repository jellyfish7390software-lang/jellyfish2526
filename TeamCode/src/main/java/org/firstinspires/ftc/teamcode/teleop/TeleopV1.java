package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LoopAction;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TeleopV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setShooterVelocity(0);
        bot.intakePower(0);
        bot.setGamepads(gamepad1, gamepad2);

        VisionPortal tag = new VisionPortal.Builder()
                .addProcessor(bot.tagProcessor)
                .setCamera(bot.tagCam)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        Robot.runCheckLoop = false;
        Robot.runScoringLoop = true;
        boolean shooterOn = false;
        boolean closeMode = true;

        int bCount = 0;

        tag.stopStreaming();

        while (opModeIsActive()) {
            bot.scoringLoopTele();
            bot.arcadeDrive(gamepad1);


            if (gamepad1.xWasPressed()) {
                if (!shooterOn) {
                    shooterOn = true;
                    closeMode = true;
                } else {
                    shooterOn = false;
                }
            }

            if (gamepad1.yWasPressed()) {
                if (!shooterOn) {
                    shooterOn = true;
                    closeMode = false;
                } else {
                    shooterOn = false;
                }
            }

            if (shooterOn) {
                bot.setShooterVelocity(closeMode ? Robot.closeRPM : Robot.farRPM);
            } else {
                bot.setShooterVelocity(0);
            }

            if (gamepad1.bWasPressed() && shooterOn ) {
                Robot.runCheckLoop = true;
                Actions.runBlocking(bot.shootFull(telemetry));
            }
//            if (bCount % 3 == 1) {
//                bot.intakePower(1);
//            }
//            if (bCount % 3 == 2) {
//                bot.intakePower(0);
//            }

            if (gamepad1.right_trigger > 0) {
                bot.intakePower(1);
                bot.checkTransferTele();
            }
            else {
                bot.intakePower(0);
            }
            if (gamepad1.left_trigger > 0) {
                bot.intakePower(-gamepad1.left_trigger);
            }
            if (gamepad1.a) {
                tag.resumeStreaming();
                Robot.atagAlign = true;
                if (!bot.tagProcessor.getDetections().isEmpty() && (bot.tagProcessor.getDetections().get(0).id == 20 || bot.tagProcessor.getDetections().get(0).id
                        == 24)) {
                    telemetry.addData("Pitch", bot.tagProcessor.getDetections().get(0).ftcPose.pitch);
                }

            }
            else {
                tag.stopStreaming();
                Robot.atagAlign = false;
            }


            telemetry.addData("Vel", Robot.rpm);
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("ShooterPower", bot.shooter.getPower());
            telemetry.addData("runCheckLoop", Robot.runCheckLoop);
            telemetry.addData("BallDist", Robot.ballDist);
            telemetry.addData("timer", bot.timer.seconds());
            telemetry.addData("IntakeDistance", bot.intakeDistance.getDistance(DistanceUnit.MM));
            telemetry.addData("BallCount", Robot.ballCount);
            telemetry.addData("ShouldTurn", Robot.ShouldTurn);
            telemetry.update();
        }

    }
}
