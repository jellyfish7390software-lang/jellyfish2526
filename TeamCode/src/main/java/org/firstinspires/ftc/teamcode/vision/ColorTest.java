package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Disabled
@Autonomous
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        int[] portals = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(0, 120, 320, 240))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)
                .build();

        AprilTagProcessor tagSensor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal color = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setLiveViewContainerId(portals[0])
                .setCamera(bot.ballCam)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        VisionPortal tag = new VisionPortal.Builder()
                .addProcessor(tagSensor)
                .setLiveViewContainerId(portals[1])
                .setCamera(bot.tagCam)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        while (opModeIsActive() || opModeInInit()) {
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            for (AprilTagDetection detection: tagSensor.getDetections()) {
                if (detection.metadata != null) {
                    telemetry.addData("DetectedID", detection.id);
                }
            }

            telemetry.addData("Best Match", result.closestSwatch);

            telemetry.update();
        }

    }
}
