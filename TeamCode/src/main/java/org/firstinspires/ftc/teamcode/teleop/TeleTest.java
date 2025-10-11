package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class TeleTest extends LinearOpMode {

    public static double targetVel = 0;

    public double vel;

    public static double ticksPerRev = 27.0;

    public static double p = 10, i = 0, d = 0, f = 12;

    public static double power = 0.0;

    public static double shooterPower = 0.0;

    public static double diverterPos = 0;

    public static double transferPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);
//
//            vel = bot.shooter.getVelocity() / ticksPerRev;
//
//            bot.shooter.setVelocity((targetVel / 60.0) * ticksPerRev);
//
            bot.leftIntake.setPower(-power);
            bot.rightIntake.setPower(-power);
//
            bot.diverter.setPosition(diverterPos);
//
            bot.shooter.setPower(shooterPower);

            bot.transfer.setPower(transferPower);

            telemetry.addData("Velocity: RPM", vel*60);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("Transfer Power: ", transferPower);
            telemetry.update();

            
        }


    }
}
