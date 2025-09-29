package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        leftIntake = hardwareMap.get(DcMotorEx.class, "left");
        rightIntake = hardwareMap.get(DcMotorEx.class, "right");
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//
//        diverter = hardwareMap.get(Servo.class, "diverter");

        ballCam = hardwareMap.get(WebcamName.class, "ballCam");
//        tagCam = hardwareMap.get(WebcamName.class, "tagCam");

        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -0.75 * gamepad1.right_stick_x;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y, x), rx));
    }

    public void arcadeDriveWithSlowMode(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = 0.5*gamepad.left_stick_y;
            x = -0.5*gamepad.left_stick_x;
            rx = -0.5*gamepad.right_stick_x;
        }
        else {
            y = gamepad.left_stick_y;
            x = -gamepad.left_stick_x;
            rx = -0.75*gamepad.right_stick_x;
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), rx));
    }
}
