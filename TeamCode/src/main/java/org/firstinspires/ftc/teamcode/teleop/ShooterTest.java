package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static double targetVel = 0;

    public double vel;

    public static double ticksPerRev = 1.0;

    public static double p = 0, i = 0, d = 0, f = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.shooter.setVelocityPIDFCoefficients(p, i, d, f);

            vel = bot.shooter.getVelocity() / ticksPerRev;

            bot.shooter.setVelocity((targetVel / 60.0) * ticksPerRev);

            telemetry.addData("Velocity: RPM", vel*60);
            telemetry.update();
        }


    }
}
