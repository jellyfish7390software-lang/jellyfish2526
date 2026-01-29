package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Maths;
import org.firstinspires.ftc.teamcode.purepursuit.math.Pose;
import org.firstinspires.ftc.teamcode.purepursuit.math.Vector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp
public class TeleopV2Blue extends LinearOpMode {
    public static boolean shooterOn, closeMode, hardstop = false, autoAim = false;
    public static boolean shooting = false;
    public static double hardstopPos = 0;
    public static boolean regression = true;
    public Pose2d pose = new Pose2d(0, 0, 0);
    public double heading = 0, error = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setPose(new Pose());
        shooterOn = false;
        closeMode = true;
        bot.hardstop.setPosition(Robot.HardstopClose);
        bot.setTelemetry(telemetry);

        AprilTagProcessor tagSensor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal tagPortal = new VisionPortal.Builder()
                .addProcessor(tagSensor)
                .setCamera(bot.camera)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        tagPortal.stopStreaming();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.scoringLoopTele();
            bot.arcadeDrive(gamepad1);
            bot.setGamepads(gamepad1, gamepad2);
            bot.purePursuit.updateSearchRadius(bot.purePursuit.searchRad);
            bot.drive.updatePoseEstimate();
            if (gamepad1.xWasPressed()) {
                if (!shooterOn) {
                    shooterOn = true;
                    closeMode = true;
                    Robot.closeMode = true;
                } else {
                    shooterOn = false;
                }
            }

            if (gamepad1.yWasPressed()) {
                if (!shooterOn) {
                    shooterOn = true;
                    closeMode = false;
                    Robot.closeMode = false;
                } else {
                    shooterOn = false;
                }
            }

            if (gamepad1.bWasPressed()) bot.shootFull(telemetry);


            if (shooterOn) {
                if (closeMode) {
                    bot.hood.setPosition(0.72);
                    if (regression) {
                    double distToGoal = Maths.dist(Robot.blueGoalClose, bot.drive.localizer.getPose().position);
                    bot.setShooterVelocity((int) bot.regressShooter(Math.min(distToGoal,110)) + 65);
                    }
                    else bot.setShooterVelocity(Robot.closeRPM);
                }
                else {
                    bot.hood.setPosition(0.775);
                    bot.setShooterVelocity(Robot.farRPM);
                }
            } else {
                bot.setShooterVelocity(0);
            }

            if (gamepad2.bWasPressed()) {
                bot.drive.localizer.setPose(new Pose2d(72 - (8.75), 72 - 8.0625, Math.toRadians(90)));
            }
            if (gamepad2.dpadDownWasPressed()) regression = !regression;

            if (gamepad2.aWasPressed()) {
                autoAim = !autoAim;
            }
            if (gamepad2.xWasPressed()) Robot.turretOffset+=25;
            if (gamepad2.yWasPressed()) Robot.turretOffset-=25;
            if (autoAim) {
                pose = bot.drive.localizer.getPose();
                if (!tagSensor.getDetections().isEmpty() && gamepad2.dpad_down) {
                    AprilTagDetection tag = tagSensor.getDetections().get(0);
                    if (tag.metadata != null && tag.id == 20) {
                        double bearing = tag.ftcPose.bearing;
                        double currentTurretDeg = bot.turretPos * Robot.ticksToDegrees;
                        double targetTurretDeg = currentTurretDeg + bearing;
                        Robot.turretTarget -= Maths.clamp(targetTurretDeg / Robot.ticksToDegrees, -630, 630);
                    }
                }

                else {
                    double goalHeading;
                    if (!closeMode)  goalHeading = Math.atan2(Robot.blueGoalFar.y - pose.position.y, Robot.blueGoalFar.x - pose.position.x);
                    else goalHeading = Math.atan2(Robot.blueGoalClose.y - pose.position.y, Robot.blueGoalClose.x - pose.position.x);
                    heading = goalHeading - pose.heading.toDouble();
                    heading = Math.atan2(Math.sin(heading), Math.cos(heading));
                    Robot.turretTarget = Maths.clamp((Math.toDegrees(heading) / Robot.ticksToDegrees), -630, 630) + Robot.turretOffset;
                }
            }

            if (gamepad1.right_trigger > 0) {
                bot.intakePower(1);
                bot.transferPower(1);
            }
            else if (gamepad1.left_trigger > 0) {
                bot.intakePower(-1);
                bot.transferPower(-1);
            }
            else if (gamepad1.right_trigger <= 0 && gamepad1.left_trigger <= 0 && !shooting) {
                bot.intakePower(0);
                bot.transferPower(0);
            }

            telemetry.addData("Ticks", bot.shooter.getCurrentPosition());
            telemetry.addData("dt", bot.dt);
            telemetry.addData("Filtered RPM", Robot.rpm);
            telemetry.addData("Target Velocity", Robot.targetVel);
            telemetry.addData("Shooter Power", bot.shooter.getPower());
            telemetry.addData("TurretPos", bot.turret.getCurrentPosition());
            telemetry.addData("Turret Target", Robot.turretTarget);
            telemetry.addData("BotX", pose.position.x);
            telemetry.addData("BotY", pose.position.y);
            telemetry.addData("BotH", pose.heading.toDouble());
            telemetry.addData("HeadingError", error);
            telemetry.addData("Heading", heading);
            telemetry.addData("Offset", Robot.turretOffset);
            telemetry.update();

        }
    }
}
