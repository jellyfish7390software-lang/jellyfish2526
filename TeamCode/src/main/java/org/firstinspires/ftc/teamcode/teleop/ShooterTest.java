package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
@Disabled
public class ShooterTest extends LinearOpMode {

    public static double targetVel = 0;

    public static double ticksPerRev = 8192.0;

    double vel = 0;

    public static double p = 0.002, i = 0, d = 0, f = 0.000265;

    public static double scale = 0;

    //13.4 V, 0.000265
    //12.47V, 0.0002750
    //12.31V, 0.00028
    //12.21, 0.000285


    //12.5, 0.000265
    //12.1, 0.0003
    public static double lastP = p, lastI = i, lastD = d, lastF = f;

    public static double tP = 0.0015, tI = 0, tD = 0;

    public static int transferPos = 0;
    public PIDController transferPID;
    public PIDFController shooterPID = new PIDFController(p, i, d, f);

    public static double power = 0.0;
    public static double shooterPower = 0.0;
    public static double transferPower = 0.0;

    public double thisTicks = 0, lastTicks = 0;

    // LPF settings
    public static double FILTER_CUTOFF = 5;   // Hz (adjustable in dashboard)
    private double filteredTicksPerSec = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        shooterPID.setPIDF(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        transferPID = new PIDController(tP, tI, tD);

        double negatived = vel;

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();
        double dt = timer.seconds();

        lastTicks = bot.shooter.getCurrentPosition();

        while (opModeIsActive() && !isStopRequested()) {

            thisTicks = -bot.shooter.getCurrentPosition();

            if (p != lastP || i != lastI || d != lastD || f != lastF) {
                shooterPID.setPIDF(p, i, d, f);
                lastP = p;
                lastI = i;
                lastD = d;
                lastF = f;
            }

            double vel = bot.shooter.getVelocity() * (60/ticksPerRev);
            negatived = Math.abs(vel);


            // Raw speed calculation
            double rawTicksPerSec = (thisTicks - lastTicks) / dt;

            double rawRPM = rawTicksPerSec *(60/ticksPerRev);

            // Low Pass Filter (RC filter)
            double RC = 1.0 / (2 * Math.PI * FILTER_CUTOFF);
            double alpha = dt / (RC + dt);
            filteredTicksPerSec = filteredTicksPerSec + alpha * (rawTicksPerSec - filteredTicksPerSec);

            double filteredRPM = filteredTicksPerSec / ticksPerRev * 60.0;

            double shooterPower = shooterPID.calculate(filteredRPM, targetVel);
            bot.shooter.setPower(bot.batteryScale(shooterPower));

            bot.intake.setPower(power);

            transferPID.setPID(tP, tI, tD);
            transferPower = transferPID.calculate(bot.transfer.getCurrentPosition(), (8192/3)*transferPos);
            bot.transfer.setPower(transferPower);

            telemetry.addData("Filtered RPM", filteredRPM);
            telemetry.addData("Velocity: RPM", vel);
            telemetry.addData("Negatived", negatived);
            telemetry.addData("RawRPM", rawRPM);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Raw Ticks/Sec", rawTicksPerSec);
            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("Transfer Target", transferPos);
            telemetry.addData("TransferPos", bot.transfer.getCurrentPosition());
            telemetry.update();
            lastTicks = thisTicks;
            dt = timer.seconds();
            timer.reset();
        }
    }
}
