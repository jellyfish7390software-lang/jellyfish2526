package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;

@Config
@TeleOp
public class SquidTuner extends LinearOpMode {
    public static double kSQx = 0.5;
    public static double kSQy = 0.75;

    public static double hP = 1.0, hI = 0, hD = 0.05;

    public static double targetX = 0, targetY = 0, targetH = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.setPose(new Pose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.purePursuit.kSQx = kSQx;
            bot.purePursuit.kSQy = kSQy;
            bot.purePursuit.hPID = new PIDCoefficients(hP, hI, hD);
            bot.purePursuit.singlePIDtoPoint(new Pose(targetX, targetY, targetH));

            telemetry.addData("TargetX", targetX);
            telemetry.addData("TargetY", targetY);
            telemetry.addData("TargetH", targetH);

            telemetry.addData("PosX", bot.purePursuit.pose.x);
            telemetry.addData("PosY", bot.purePursuit.pose.y);
            telemetry.addData("PosH", bot.purePursuit.pose.h);

            telemetry.update();
        }
    }
}
