package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {

    public MecanumDrive drive;
    public DcMotorEx leftIntake, rightIntake, shooter;
    public Servo diverter;
    public WebcamName ballCam, tagCam;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

//        leftIntake = hardwareMap.get(DcMotorEx.class, "left");
//        rightIntake = hardwareMap.get(DcMotorEx.class, "right");
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//
//        diverter = hardwareMap.get(Servo.class, "diverter");

        ballCam = hardwareMap.get(WebcamName.class, "ballCam");
        tagCam = hardwareMap.get(WebcamName.class, "tagCam");

    }
}
