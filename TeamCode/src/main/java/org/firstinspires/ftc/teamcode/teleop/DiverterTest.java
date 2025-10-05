package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Robot;


import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class DiverterTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        Robot bot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //bot.diverter.setPower((double) (gamepad1.right_stick_y));
            bot.diverter.setPower(0.7);

        }
    }
}
