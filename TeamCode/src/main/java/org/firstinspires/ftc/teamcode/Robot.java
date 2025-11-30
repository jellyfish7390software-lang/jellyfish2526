package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.actions.LoopAction;
import org.firstinspires.ftc.teamcode.actions.PurePursuitAction;
import org.firstinspires.ftc.teamcode.actions.WaitUntilAction;
import org.firstinspires.ftc.teamcode.comp1.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Path;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;
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

    public PurePursuit purePursuit;

    public Telemetry telemetry;

    public Gamepad gamepad1, gamepad2;
    public VoltageSensor voltage;

    public static double p = 0.002, i = 0, d = 0, f = 0.00024;
    public static double lastP = p, lastI = i, lastD = d, lastF = f;

    public static double FILTER_CUTOFF = 5;   // Hz (adjustable in dashboard)
    private double filteredTicksPerSec = 0;
//    public static double tP = -0.001, tI = 0, tD = 0;

    public static double tP = 0.00085, tI = 0, tD = 0;
    public static double hP = 0.04, hI = 0, hD = 0;

    public static double leftOffset = -1.5, rightOffset = 1;
    public PIDController hPID = new PIDController(hP, hI, hD);
    public static boolean atagAlign = false;

    public static double ticksPerRev = 8192.0;

    public double lastTicks = 0, thisTicks = 0;

    public static double ballDist = 0;

    public static double targetVel = 0;
    public static int transferTarget = 0;
    public static double intakePower = 0;

    public static double rpm = 0;

    public double lastTime = 0;

    public static boolean runScoringLoop = true;
    public static boolean runCheckLoop = false;

    public PIDController transferPID;
    public PIDFController shooterPID = new PIDFController(p, i, d, f);

    public static double TAG_HEIGHT = 38.75-9.25;
    public static Vector2d RED_GOAL_TAG = new Vector2d(-58.27, 55.63);
    public static Vector2d BLUE_GOAL_TAG = new Vector2d(-58.27, -55.63);

    public static int closeRPM = 2900;
    public static int farRPM = 3750;

    public static int ballCount = 0;
    public static boolean ShouldTurn = false;

    public static double heading;

    public ElapsedTime timer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        targetVel = 0;
        transferTarget = 0;
        intakePower = 0;
        ballDist = 0;
        ballCount = 0;
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        purePursuit = new PurePursuit(drive);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        voltage = hardwareMap.voltageSensor.iterator().next();

//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }

//        TODO: (10/31) Add correct config name once mounted
        intakeDistance = (DistanceSensor) hardwareMap.get(ColorSensor.class, "intakeDistance");

        tagCam = hardwareMap.get(WebcamName.class, "tagCam");
//
//        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
//
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        transferPID = new PIDController(tP, tI, tD);
        shooterPID.setPIDF(p, i, d, f);

        timer.reset();

        lastTicks = shooter.getCurrentPosition();

    }


    public void setPose(Pose pose) {
        drive.localizer.setPose(pose.toPose2d());
        purePursuit.mecDrive = drive;
    }
    public Action followPath(Path path) {
        return new PurePursuitAction(purePursuit, path);
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
    public double batteryScale(double num) {
        return (12.5/voltage.getVoltage()) * num;
    }
    public void scoringLoopTele() {
        transferPID.setPID(tP, tI, tD);

        if (p != lastP || i != lastI || d != lastD || f != lastF) {
            shooterPID.setPIDF(p, i, d, f);
            lastP = p;
            lastI = i;
            lastD = d;
            lastF = f;
        }

        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 0) dt = 0.001;

        thisTicks = -shooter.getCurrentPosition();

        double rawTicksPerSec = (thisTicks - lastTicks) / dt;
        double rawRPM = rawTicksPerSec * (60 / ticksPerRev);

        double RC = 1.0 / (2 * Math.PI * FILTER_CUTOFF);
        double alpha = dt / (RC + dt);
        filteredTicksPerSec += alpha * (rawTicksPerSec - filteredTicksPerSec);
        double filteredRPM = filteredTicksPerSec / ticksPerRev * 60.0;

        rpm = filteredRPM;

        double shooterPower = shooterPID.calculate(filteredRPM, targetVel);
        shooter.setPower(batteryScale(shooterPower));

        double transferPower = transferPID.calculate(transfer.getCurrentPosition(), transferTarget);
        transfer.setPower(transferPower);

        intakePower(intakePower);

        lastTicks = thisTicks;
    }

    public class ScoringLoop implements Action {
        public double dt;
        public ScoringLoop() {
            timer.reset();
            dt = timer.seconds();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            transferPID.setPID(tP, tI, tD);

            if (p != lastP || i != lastI || d != lastD || f != lastF) {
                shooterPID.setPIDF(p, i, d, f);
                lastP = p;
                lastI = i;
                lastD = d;
                lastF = f;
            }

            double now = timer.seconds();
            double dt = now - lastTime;
            lastTime = now;
            if (dt <= 0) dt = 0.001;

            thisTicks = -shooter.getCurrentPosition();

            double rawTicksPerSec = (thisTicks - lastTicks) / dt;
            double rawRPM = rawTicksPerSec * (60 / ticksPerRev);

            double RC = 1.0 / (2 * Math.PI * FILTER_CUTOFF);
            double alpha = dt / (RC + dt);
            filteredTicksPerSec += alpha * (rawTicksPerSec - filteredTicksPerSec);
            double filteredRPM = filteredTicksPerSec / ticksPerRev * 60.0;

            rpm = filteredRPM;

            double shooterPower = shooterPID.calculate(filteredRPM, targetVel);
            shooter.setPower(batteryScale(shooterPower));

            double transferPower = transferPID.calculate(transfer.getCurrentPosition(), transferTarget);
            transfer.setPower(transferPower);

            intakePower(intakePower);

            lastTicks = thisTicks;

            return Robot.runScoringLoop;
        }
    }
    public void turnTransfer() {
        transferTarget += 8192/3;
    }
    public Action turnTransferAction() {
        return new InstantAction(() -> transferTarget += 8192/3);
    }
    public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    public Action autoCheckTransfer() {
        return new LoopAction(() -> {
            if (runCheckLoop) checkTransferTele();
        });
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
    public Action waitUntilReady(Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(new WaitUntilAction(() -> Math.abs(getRpm() - targetVel) <30), scoringLoop(), driveAction(gamepad), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
        }));
    }
    public Action waitUntilReady() {
        return new RaceAction(new WaitUntilAction(() -> Math.abs(getRpm() - targetVel) < 30), scoringLoop());
    }
    public Action waitForIntake(Gamepad gamepad, Telemetry telemetry) {
        return new RaceAction(scoringLoop(), driveAction(gamepad), telemetryPacket -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <100);
            telemetry.update();
            return Math.abs(getRpm() - targetVel) > 100;
        });
    }
    public Action waitForIntake(Telemetry telemetry) {
        return new RaceAction(scoringLoop(),telemetryPacket -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
            return Math.abs(getRpm() - targetVel) > 10;});
    }
    /// New
    public Action shootFull(Telemetry telemetry) {
        return new SequentialAction(
                turnTransferAction(),
                sleepWithPIDTeleop(0.75, gamepad1, telemetry),
                new InstantAction(() -> targetVel -= 20),
                new InstantAction(() -> intakePower(1)),
                waitForDriver(gamepad1, telemetry),
                turnTransferAction(),
                sleepWithPIDTeleop(0.75, gamepad1, telemetry),
                waitForDriver(gamepad1, telemetry),
                turnTransferAction(),
                new InstantAction(() -> ballCount = 0));
    }
    public Action shootFullAuto(Telemetry telemetry) {
        return new RaceAction(new SequentialAction(
                waitForIntake(telemetry),
                turnTransferAction(),
                new InstantAction(() -> targetVel += 28),
                new InstantAction(() -> intakePower(1)),
                sleepWithPID(0.5),
                waitForIntake(telemetry),
                turnTransferAction(),
                new InstantAction(() -> targetVel -= 48),
                sleepWithPID(0.5),
                waitForIntake(telemetry),
                turnTransferAction(),
                new InstantAction(() -> ballCount = 0)),
                new LoopAction(() -> {
                    telemetry.addData("Vel", getRpm());
                    telemetry.addData("Target", Robot.targetVel);
                    telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
                    telemetry.update();
                }));
    }
    public Action shootFullAutoFar(Telemetry telemetry) {
        return new RaceAction(new SequentialAction(
//                new InstantAction(() -> targetVel += 50),
                waitForIntake(telemetry),
                turnTransferAction(),
                new InstantAction(() -> intakePower(1)),
                sleepWithPID(0.5),
//                new InstantAction(() -> targetVel -= 50),
                waitForIntake(telemetry),
                turnTransferAction(),
                sleepWithPID(0.5),
                waitForIntake(telemetry),
                turnTransferAction(),
                new InstantAction(() -> ballCount = 0)),
                new LoopAction(() -> {
                    telemetry.addData("Vel", getRpm());
                    telemetry.addData("Target", Robot.targetVel);
                    telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
                    telemetry.update();
                }));
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
    public Action waitForDriver(Gamepad gamepad1, Telemetry telemetry) {
        return new RaceAction(new WaitUntilAction(() -> gamepad1.bWasPressed() && Math.abs(getRpm() - Robot.targetVel) < 40 || gamepad2.yWasPressed()), scoringLoop(), driveAction(gamepad1), new LoopAction(() -> {
            telemetry.addData("Vel", getRpm());
            telemetry.addData("Target", Robot.targetVel);
            telemetry.addData("inRange", Math.abs(getRpm() - targetVel) <30);
            telemetry.update();
        }));
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        heading = 0;
        if (!atagAlign) {
            double rx = -0.75 * gamepad1.right_stick_x;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y, x), rx));
        }
        if (atagAlign) {
            atagAlign(-y, x);
        }
        if (gamepad1.dpad_up) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 0),0));
        }
        if (gamepad1.dpad_down) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1, 0),0));
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            heading = drive.localizer.getPose().heading.toDouble();
        }
        if (gamepad1.dpad_left) {
            double hPower = hPID.calculate(drive.localizer.getPose().heading.toDouble(), heading);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 1),hPower));
        }
        if (gamepad1.dpad_right) {
            double hPower = hPID.calculate(drive.localizer.getPose().heading.toDouble(), heading);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -1), hPower));
        }
    }
    public double getRpm() {
        return rpm;
    }
    public void atagAlign(double x, double y) {
        hPID.setPID(hP, hI, hD);
        double hPower = 0;
        if (!tagProcessor.getDetections().isEmpty() && (tagProcessor.getDetections().get(0).id == 20)) {
            hPower = hPID.calculate(tagProcessor.getDetections().get(0).ftcPose.bearing, 0);
        }
        else if (!tagProcessor.getDetections().isEmpty() && tagProcessor.getDetections().get(0).id
                == 24) {
            hPower = hPID.calculate(tagProcessor.getDetections().get(0).ftcPose.bearing, 0);
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
