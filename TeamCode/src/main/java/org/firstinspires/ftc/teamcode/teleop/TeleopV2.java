package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.actions.LoopAction;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;

@Config
@TeleOp
public class TeleopV2 extends LinearOpMode {
    public static boolean shooterOn, closeMode, hardstop = false;
    public static boolean shooting = false;
    public static double hardstopPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose());
        shooterOn = false;
        closeMode = true;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.scoringLoopTele();
            bot.arcadeDrive(gamepad1);
            bot.setGamepads(gamepad1, gamepad2);
            bot.purePursuit.updateSearchRadius(bot.purePursuit.searchRad);
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

            if (gamepad1.bWasPressed()) bot.shootFull(telemetry);


            if (shooterOn) {
                bot.setShooterVelocity(closeMode ? Robot.closeRPM : Robot.farRPM);
            } else {
                bot.setShooterVelocity(0);
            }

            if (gamepad1.right_trigger > 0) {
                bot.intakePower(1);
                bot.transferPower(1);
            }
            else if (gamepad1.left_trigger > 0) {
                bot.intakePower(-1);
                bot.transferPower(-1);
            }
            else if (gamepad1.right_trigger <= 0 && gamepad1.left_trigger <= 0 && !shooting) {
                bot.intakePower(0);
                bot.transferPower(0);
            }

            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Filtered RPM", Robot.rpm);
            telemetry.addData("Target Velocity", Robot.targetVel);
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("TurretPos", bot.turret.getCurrentPosition());
            telemetry.addData("Turret Target", Robot.turretTarget);
            telemetry.update();

        }
    }
}
