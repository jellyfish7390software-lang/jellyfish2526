package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class IndexerTest extends LinearOpMode {

public static double power = 0.7;

    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //bot.diverter.setPower((double) (gamepad1.right_stick_y));
            bot.indexer.setPower(power);

            telemetry.addData("Power: ", power);
            telemetry.update();

        }
    }
}
