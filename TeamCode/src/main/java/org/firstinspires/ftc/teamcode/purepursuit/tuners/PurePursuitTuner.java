package org.firstinspires.ftc.teamcode.purepursuit.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;


@TeleOp
@Config
public class PurePursuitTuner extends LinearOpMode {
    public Bezier curve = PurePursuit.builder
            .addControlPoint(0, 0)
//            .addControlPoint(48, 0)
            .addControlPoint(0, 48)
            .addControlPoint(48,48)
            .build();
    public Bezier curve2 = PurePursuit.builder
            .addControlPoint(48,48)
            .addControlPoint(48,0)
            .addControlPoint(0,0)
            .build();
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Actions.runBlocking(bot.followPath(curve));
            Actions.runBlocking(bot.followPath(curve2));
        }

    }
}
