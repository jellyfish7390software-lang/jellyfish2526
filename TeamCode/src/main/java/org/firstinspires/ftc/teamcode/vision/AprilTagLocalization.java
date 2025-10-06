package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class AprilTagLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, -48, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VisionPortal tag = new VisionPortal.Builder()
                .addProcessor(bot.tagProcessor)
                .setCamera(bot.tagCam)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("X", bot.getBotPose(drive).position.x);
            telemetry.addData("Y", bot.getBotPose(drive).position.y);
            telemetry.addData("H", Math.toDegrees(bot.getBotPose(drive).heading.toDouble()));
            telemetry.update();
        }
    }
}
