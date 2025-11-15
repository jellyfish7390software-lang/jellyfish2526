package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static double targetVel = 0;

    public double vel;

    public static double ticksPerRev = 27.0;

    public static double p = 10, i = 0, d = 0, f = 12;
    public static double tP = 0.0015, tI = 0, tD = 0;

    public static int transferPos = 0;
    public PIDController transferPID;

    public static double power = 0.0;
    public static double shooterPower = 0.0;
    public static double transferPower = 0.0;

    public static double diverterPos = 0;

    /// 13.64 V / 11.9
    ///

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        transferPID = new PIDController(tP, tI, tD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

            vel = bot.shooter.getVelocity() / ticksPerRev;

            bot.shooter.setVelocity((targetVel / 60.0) * ticksPerRev);

            bot.intake.setPower(power);

//            bot.transfer.setPower(transferPower);

            transferPID.setPID(tP, tI, tD);

            transferPower = transferPID.calculate(bot.transfer.getCurrentPosition(), transferPos);
            bot.transfer.setPower(transferPower);

//            bot.diverter.setPosition(diverterPos);

//            bot.shooter.setPower(shooterPower);

            telemetry.addData("Velocity: RPM", vel*60);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("Transfer Target: ", transferPos);
            telemetry.addData("TransferPos", bot.transfer.getCurrentPosition());
            telemetry.update();
        }


    }
}
