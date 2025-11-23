package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double leftPower = 1;
    public static double rightPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            bot.leftIntake.setPower(leftPower);
            bot.rightIntake.setPower(-leftPower);
        }

    }
}
