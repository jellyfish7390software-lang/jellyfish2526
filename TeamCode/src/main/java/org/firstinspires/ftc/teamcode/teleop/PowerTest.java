package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp
@Config
public class PowerTest extends LinearOpMode {
    public static double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        waitForStart();
        while(opModeIsActive())
        {
            shooter.setPower(power);
        }
    }
}
