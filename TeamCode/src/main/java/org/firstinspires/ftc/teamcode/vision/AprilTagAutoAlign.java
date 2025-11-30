package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.comp1.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@TeleOp
@Config
public class AprilTagAutoAlign extends LinearOpMode {
    public static double hP = 0.05, hI = 0, hD = 0;
    public PIDController hPID;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        hPID = new PIDController(hP, hI, hD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VisionPortal tag = new VisionPortal.Builder()
                .addProcessor(bot.tagProcessor)
                .setCamera(bot.tagCam)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            hPID.setPID(hP, hI, hD);
            double hPower = 0;
            if (!bot.tagProcessor.getDetections().isEmpty() && (bot.tagProcessor.getDetections().get(0).id == 20 || bot.tagProcessor.getDetections().get(0).id
            == 24)) {
                hPower = hPID.calculate(bot.tagProcessor.getDetections().get(0).ftcPose.bearing, 0);
                telemetry.addData("Pitch", bot.tagProcessor.getDetections().get(0).ftcPose.bearing);
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), -hPower));

            drive.updatePoseEstimate();
            telemetry.addData("X", bot.getBotPose(drive).position.x);
            telemetry.addData("Y", bot.getBotPose(drive).position.y);
            telemetry.addData("H", Math.toDegrees(bot.getBotPose(drive).heading.toDouble()));
            telemetry.addData("ActualX", drive.localizer.getPose().position.x);
            telemetry.addData("ActualY", drive.localizer.getPose().position.y);
            telemetry.addData("ActualH", drive.localizer.getPose().heading.toDouble());
            telemetry.update();
        }
    }
}
