package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class TeleTest extends LinearOpMode {

    public static double targetVel = 0;

    public double vel;

    public static double ticksPerRev = 27.0;

    public static double p = 10, i = 0, d = 0, f = 13.25;

    public static double tP = -0.001, tI = 0, tD = 0;

    public static double power = 1.0;

    public static double shooterPower = 0.0;

   // public static double diverterPos = 0;

    public static int shootingVel = 3900;
    public static int transferPos = 0;
    public static double transferPower = 0.0;
    public PIDController transferPID;
    int bIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        transferPID = new PIDController(tP, tI, tD);

        bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.arcadeDrive(gamepad1);

            transferPID.setPID(tP, tI, tD);

            if (gamepad1.right_trigger > 0) targetVel = shootingVel;
            else targetVel = 0;

            bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

            vel = bot.shooter.getVelocity() / ticksPerRev;

            bot.shooter.setVelocity((targetVel / 60.0) * ticksPerRev);

            if (gamepad1.bWasPressed()) {
                bIndex++;
            }

            if (bIndex % 2 == 1) {
                bot.leftIntake.setPower(-power);
                bot.rightIntake.setPower(power);
            }
            else if (bIndex % 2 == 0) {
                bot.leftIntake.setPower(0);
                bot.rightIntake.setPower(0);
            }

            if (gamepad1.xWasPressed()) {
                transferPos += 8192/3;
            }

            transferPower = transferPID.calculate(bot.transfer.getCurrentPosition(), -transferPos);
            bot.transfer.setPower(transferPower);

          //  bot.diverter.setPosition(diverterPos);
//
            bot.shooter.setPower(shooterPower);

            telemetry.addData("Velocity: RPM", vel*60);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("TransferPos: ", bot.transfer.getCurrentPosition());
            telemetry.addData("Transfer Target", transferPos);
            telemetry.update();

        }


    }
}
