package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp
public class TransferTest extends LinearOpMode {

    public static double tP = 0.003, tI = 0, tD = 0;

    public static int transferPos = 0;
    public double transferPower = 0.0;
    public PIDController transferPID;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        transferPID = new PIDController(tP, tI, tD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            transferPID.setPID(tP, tI, tD);

            transferPower = transferPID.calculate(bot.transfer.getCurrentPosition(), transferPos);
            bot.transfer.setPower(transferPower);

            telemetry.addData("Transfer Target: ", transferPos);
            telemetry.addData("TransferPos", bot.transfer.getCurrentPosition());
            telemetry.update();
        }
    }
}
