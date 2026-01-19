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
public class ShooterTest extends LinearOpMode {

    public static double targetVel = 0;
    public static double turretTarget = 0;
    public static double ticksPerRev = 8192.0;
    public static double hoodPos = 0;

    double vel = 0;

    public static double p = 0.02, i = 0, d = 0, f = 0.000205;
    public static double scale = 0;
    public static double lastP = p, lastI = i, lastD = d, lastF = f;

    public static double tP = 0.0015, tI = 0, tD = 0;
    public static double turretP = 0.01, turretI = 0, turretD = 0;

    public static double intakePower = 0.0;

    public static int transferPos = 0;
    public PIDController transferPID;
    public PIDFController shooterPID = new PIDFController(p, i, d, f);
    public PIDController turretPID = new PIDController(turretP, turretI, turretD);

    public static double power = 0.0;
    public static double shooterPower = 0.0;
    public static double transferPower = 0.0;
    public static double turretPower = 0.0;

    public double thisTicks = 0, lastTicks = 0;

    // LPF settings
    public static double FILTER_CUTOFF = 3;   // Hz (adjustable in dashboard)
    private double filteredTicksPerSec = 0;

    private double tps0 = 0;
    private double tps1 = 0;
    private double tps2 = 0;

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

            thisTicks = bot.shooter.getCurrentPosition();

//            f = bot.regressF(targetVel);

            if (p != lastP || i != lastI || d != lastD || f != lastF) {
                shooterPID.setPIDF(p, i, d, f);
                lastP = p;
                lastI = i;
                lastD = d;
                lastF = f;
            }

            double vel = bot.shooter.getVelocity() * (60 / ticksPerRev);
            negatived = Math.abs(vel);

            // Raw speed calculation
            double rawTicksPerSec = (thisTicks - lastTicks) / dt;
            double rawRPM = rawTicksPerSec * (60 / ticksPerRev);

            // -------- MEDIAN FILTER (ADDED) --------
            tps0 = tps1;
            tps1 = tps2;
            tps2 = rawTicksPerSec;

            double medianTicksPerSec = Math.max(
                    Math.min(tps0, tps1),
                    Math.min(Math.max(tps0, tps1), tps2)
            );
            // -------------------------------------

            // Low Pass Filter (RC filter)
            double RC = 1.0 / (2 * Math.PI * FILTER_CUTOFF);
            double alpha = dt / (RC + dt);
            filteredTicksPerSec = filteredTicksPerSec
                    + alpha * (medianTicksPerSec - filteredTicksPerSec);

            double filteredRPM = filteredTicksPerSec / ticksPerRev * 60.0;

            double shooterPower = shooterPID.calculate(filteredRPM, targetVel);
            if (targetVel == 0) {
                shooterPower = 0;
            }
            bot.shooter.setPower(shooterPower);

            if (power == 0) {
                bot.intake.setPower(intakePower);
                bot.transfer.setPower(transferPower);
            }
            else {
                bot.intake.setPower(power);
                bot.transfer.setPower(power);
            }

            transferPID.setPID(tP, tI, tD);
            transferPower = transferPID.calculate(
                    bot.transfer.getCurrentPosition(),
                    (8192 / 3) * transferPos
            );

            turretPID.setPID(turretP, turretI, turretD);
            turretPower = turretPID.calculate(
                    bot.turret.getCurrentPosition(),
                    turretTarget
            );
            bot.turret.setPower(turretPower);
            bot.hood.setPosition(hoodPos);


            telemetry.addData("Filtered RPM", filteredRPM);
            telemetry.addData("Velocity: RPM", vel);
            telemetry.addData("Negatived", negatived);
            telemetry.addData("RawRPM", rawRPM);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Raw Ticks/Sec", rawTicksPerSec);
            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("TurretPos", bot.turret.getCurrentPosition());
            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("TurretPower", turretPower);
            telemetry.addData("Transfer Target", transferPos);
            telemetry.addData("TransferPos", bot.transfer.getCurrentPosition());
            telemetry.update();

            lastTicks = thisTicks;
            dt = timer.seconds();
            timer.reset();
        }
    }
}
