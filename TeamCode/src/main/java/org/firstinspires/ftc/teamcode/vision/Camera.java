package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp
public class Camera extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor myAprilTagProcessor;
// Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        while (opModeIsActive() && (!isStopRequested())) {
            if (!myAprilTagProcessor.getDetections().isEmpty()) {
                telemetry.addData("bearing", myAprilTagProcessor.getDetections().get(0).ftcPose.bearing);
                if (myAprilTagProcessor.getDetections().get(0).ftcPose.bearing < 0)
                    telemetry.addLine("turn right");

                if (myAprilTagProcessor.getDetections().get(0).ftcPose.bearing > 0) {
                    telemetry.addLine("turn left");

                }
                telemetry.update();
            }


        }
    }
}
