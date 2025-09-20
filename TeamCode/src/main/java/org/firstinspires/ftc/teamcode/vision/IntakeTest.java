package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class IntakeTest extends LinearOpMode {
    DcMotorEx left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            left.setPower(0.9);
            right.setPower(0.9);
        }

    }
}
