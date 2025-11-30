package org.firstinspires.ftc.teamcode.comp1.comp1Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
@Disabled
public class PowerTest extends LinearOpMode {



    public static double power = 0.0;

    public static double shooterPower = 0.0;

    public static double transferPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        bot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {

        bot.shooter.setPower(-shooterPower); //0.6-0.7 from med
        bot.intake.setPower(power);
        bot.transfer.setPower(-transferPower);

            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("Transfer Target: ", bot.transfer.getCurrentPosition());
            telemetry.update();

        }
    }
}
