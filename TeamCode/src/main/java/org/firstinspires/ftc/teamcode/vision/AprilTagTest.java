package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor processor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), processor);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();


        while (opModeIsActive()) {

            List<AprilTagDetection> detections;
            int myAprilTagIdCode;

            detections = processor.getDetections();

            for (AprilTagDetection detection: detections) {

                if (detection.metadata != null) {
                    myAprilTagIdCode = detection.id;
                    Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Original source data
                    double poseX = detection.rawPose.x;
                    double poseY = detection.rawPose.y;
                    double poseZ = detection.rawPose.z;

                    double poseAX = rot.firstAngle;
                    double poseAY = rot.secondAngle;
                    double poseAZ = rot.thirdAngle;


                    telemetry.addData("DetectedID", myAprilTagIdCode);
                    telemetry.addData("poseX", poseX);
                    telemetry.addData("poseY", poseY);
                    telemetry.addData("poseZ", poseZ);
                    telemetry.addData("poseAX", poseAX);
                    telemetry.addData("poseAY", poseAY);
                    telemetry.addData("poseAZ", poseAZ);


                }

            }
            telemetry.update();
        }

    }

}

