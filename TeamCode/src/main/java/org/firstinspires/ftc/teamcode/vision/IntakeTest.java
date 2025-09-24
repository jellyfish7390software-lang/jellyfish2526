package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        bot.rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            bot.leftIntake.setPower(0.9);
            bot.rightIntake.setPower(0.9);
        }

    }
}
