package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionList;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        bot.setShooterVelocity(0);
        bot.intakePower(1);
        
        waitForStart();

//        Actions.runBlocking(new ParallelAction(bot.scoringLoop(), bot.checkTransfer()));

        while (opModeIsActive() && !isStopRequested()) {

            // TODO: Only run this after re-enabling the hardwareMap.get in Robot
            telemetry.addData("IntakeDistance", bot.intakeDistance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
