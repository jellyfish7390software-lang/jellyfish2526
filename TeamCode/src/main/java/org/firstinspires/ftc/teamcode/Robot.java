package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.ejml.data.DMatrixSparseCSC;
import org.ejml.dense.row.mult.MatrixMatrixMult_CDRM;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class Robot {

    public MecanumDrive drive;
    public DcMotorEx leftIntake, rightIntake, intake, shooter;
    public DcMotorEx transfer;
    public Servo diverter;
    public WebcamName ballCam, tagCam;
    public AprilTagProcessor tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    public List<AprilTagDetection> detections;
    public DistanceSensor distance, intakeDistance;

    public Gamepad gamepad1, gamepad2;

    public static double p = 10, i = 0, d = 0, f = 13.25;
//    public static double tP = -0.001, tI = 0, tD = 0;

    public static double tP = 0.003, tI = 0, tD = 0;
    public static double hP = -0.04, hI = 0, hD = 0;
    public PIDController hPID = new PIDController(hP, hI, hD);
    public static boolean atagAlign = false;

    public static double ticksPerRev = 28.0;

    public static double ballDist = 0;

    public static int targetVel = 0;
    public static int transferTarget = 0;
    public static double intakePower = 0;

    public static boolean runScoringLoop = true;
    public static boolean runCheckLoop = false;

    public PIDController transferPID;

    public static double TAG_HEIGHT = 38.75-9.25;
    public static Vector2d RED_GOAL_TAG = new Vector2d(-58.27, 55.63);
    public static Vector2d BLUE_GOAL_TAG = new Vector2d(-58.27, -55.63);

    public static int closeRPM = 3850;
    public static int farRPM = 4300;

    public static int ballCount = 0;
    public static boolean ShouldTurn = false;

    public ElapsedTime timer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        targetVel = 0;
        transferTarget = 0;
        intakePower = 0;
        ballDist = 0;
        ballCount = 0;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

//        TODO: (10/31) Add correct config name once mounted
        intakeDistance = (DistanceSensor) hardwareMap.get(ColorSensor.class, "intakeDistance");

        tagCam = hardwareMap.get(WebcamName.class, "tagCam");
//
//        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        transferPID = new PIDController(tP, tI, tD);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);

        timer.reset();

    }

//    public void purplePath() {
//        diverter.setPosition(0.8);
//    }
//    public void greenPath() {
//        diverter.setPosition(1);
//    }
    public Action scoringLoop() {
        return new ScoringLoop();
    }
    public void setShooterVelocity(int vel) {
        targetVel = vel;
    }
    public void setTransferPosition(int target) {
        transferTarget = target;
    }
    public void incrementTransfer(int increment) {
        transferTarget += increment;
    }
    public void scoringLoopTele() {
        transferPID.setPID(tP, tI, tD);
        shooter.setVelocityPIDFCoefficients(p, i, d, f);

        double vel = shooter.getVelocity() / ticksPerRev;

        shooter.setVelocity((targetVel / 60.0) * ticksPerRev);

        double transferPower = transferPID.calculate(transfer.getCurrentPosition(), transferTarget);
        transfer.setPower(transferPower);

        intakePower(intakePower);
    }

    public class ScoringLoop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transferPID.setPID(tP, tI, tD);
            shooter.setVelocityPIDFCoefficients(p, i, d, f);

            double vel = shooter.getVelocity() / ticksPerRev;

            shooter.setVelocity((targetVel / 60.0) * ticksPerRev);

//            double transferPower = transferPID.calculate(transfer.getCurrentPosition(), -transferTarget);
            double transferPower = transferPID.calculate(transfer.getCurrentPosition(), transferTarget);
            transfer.setPower(transferPower);

            Robot.ballDist = distance.getDistance(DistanceUnit.MM);

            intakePower(intakePower);

            return Robot.runScoringLoop;
        }
    }
    public void turnTransfer() {
        transferTarget += 2000/3;
    }
    public Action turnTransferAction() {
        return new InstantAction(() -> transferTarget += 2000/3);
    }
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public void checkTransferTele() {
        double intakeDist = intakeDistance.getDistance(DistanceUnit.MM);
        Robot.ballDist = distance.getDistance(DistanceUnit.MM);


        if (Robot.ballDist > 0 && Robot.ballDist < 40 && timer.seconds() > 0.75) {
            if (ballCount < 1) turnTransfer();

            timer.reset();

            ballCount++;

            if (ballCount > 2 && intakeDist > 0 && intakeDist < 15) {
                intakePower(0);
            }

        }
    }
    public class CheckTransfer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double intakeDist = intakeDistance.getDistance(DistanceUnit.MM);

            boolean shouldTurn = Robot.runCheckLoop;

            if (Robot.ballDist > 0 && Robot.ballDist < 40 && timer.seconds() > 0.75) {
                if (shouldTurn) turnTransfer();

                timer.reset();

                ballCount++;
                if (ballCount >= 1) {
                    shouldTurn = false;
                }

                if (ballCount > 2 && intakeDist > 0 && intakeDist < 15) {
                    intakePower(0);
                }

                Robot.ShouldTurn = shouldTurn;
            }
            return Robot.runCheckLoop;
        }
    }
    public Action driveAction(Gamepad gamepad) {
        return new LoopAction(() -> arcadeDrive(gamepad));
    }
    public Action sleepWithPID(double dt) {
        return new RaceAction(new SleepAction(dt), scoringLoop());
    }
    public Action sleepWithPIDTeleop(double dt, Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(new SleepAction(dt), scoringLoop(), driveAction(gamepad), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.update();
        }));
    }
    /// New
    public Action shootFull(Telemetry telemetry) {
        return new SequentialAction(turnTransferAction(),
                sleepWithPIDTeleop(0.5, gamepad1, telemetry),
                new InstantAction(() -> intakePower(1)),
                sleepWithPIDTeleop(0.75, gamepad1, telemetry),
                turnTransferAction(),
                sleepWithPIDTeleop(1, gamepad1, telemetry),
                turnTransferAction(),
                new InstantAction(() -> ballCount = 0));
    }
    public Action shootFull() {
        return new SequentialAction(turnTransferAction(),
                sleepWithPID(0.5),
                new InstantAction(() -> intakePower(1)),
                sleepWithPID(0.75),
                turnTransferAction(),
                sleepWithPID(1),
                turnTransferAction(),
                new InstantAction(() -> ballCount = 0));
    }
    /// New
    public Action checkTransfer() {
        return new CheckTransfer();
    }
    public void intakePower(double power) {
        intakePower = power;
//        intake.setPower(power);
        intake.setPower(power);
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        if (!atagAlign) {
            double rx = -0.75 * gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y, x), rx));
        }
        if (atagAlign) {
            atagAlign(-y, x);
        }
    }
    public double getRpm() {
        double vel = shooter.getVelocity() / ticksPerRev;
        return vel * 60;
    }
    public void atagAlign(double x, double y) {
        hPID.setPID(hP, hI, hD);
        double hPower = 0;
        if (!tagProcessor.getDetections().isEmpty() && (tagProcessor.getDetections().get(0).id == 20 || tagProcessor.getDetections().get(0).id
                == 24)) {
            hPower = hPID.calculate(tagProcessor.getDetections().get(0).ftcPose.pitch, 0);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x,y), -hPower));

        drive.updatePoseEstimate();
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

    //TODO: AprilTag Code

    public Pose2d getBotPose(MecanumDrive drive) {
        detections = tagProcessor.getDetections();
        if (detections.isEmpty()) return drive.localizer.getPose();

        for (AprilTagDetection tag : detections) {
            if (tag.id == 20 || tag.id == 24) {
                double r = get2dRange(tag);

                double theta = Math.toRadians(tag.ftcPose.bearing); //TODO: check if radians
                double imuHeading = drive.localizer.getPose().heading.toDouble();

                Vector2d tagPos = (tag.id == 20) ? BLUE_GOAL_TAG : RED_GOAL_TAG;

                double dx = r * Math.cos(theta + imuHeading);
                double dy = r * Math.sin(theta + imuHeading);

                double camX = tagPos.x - dx;
                double camY = tagPos.y - dy;

                double cx = 0; // forward offset from robot center
                double cy = 0; // left offset from robot center

                double offsetX = cx * Math.cos(imuHeading) - cy * Math.sin(imuHeading);
                double offsetY = cx * Math.sin(imuHeading) + cy * Math.cos(imuHeading);

                double rx = camX - offsetX;
                double ry = camY - offsetY;

                return new Pose2d(rx, ry, imuHeading);
            }
        }
        return drive.localizer.getPose();
    }


    public double get2dRange(AprilTagDetection detection) {
        if (detection.metadata != null) {
            return Math.sqrt(Math.pow(detection.ftcPose.range, 2) - Math.pow(TAG_HEIGHT, 2));
        }
        return 0;
    }

}
