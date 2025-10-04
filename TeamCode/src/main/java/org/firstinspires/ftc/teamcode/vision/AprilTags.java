package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTags extends LinearOpMode {
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
        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> myAprilTagDetections;  // list of all detections
                  // current detection in for() loop
            int myAprilTagIdCode;                           // ID code of current detection, in for() loop

// Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();

// Cycle through through the list and process each AprilTag.
            for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {

                if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    myAprilTagIdCode = myAprilTagDetection.id;

                    // Now take action based on this tag's ID code, or store info for later action.
                telemetry.addData("id", myAprilTagIdCode);
                }
            }
        telemetry.update();
        }

    }

}
