package com.example.simulator;

import com.example.simulator.math.Bezier;
import com.example.simulator.math.ParametricPose;
import com.example.simulator.math.Pose;
import com.example.simulator.math.PurePursuit;
import com.example.simulator.sim.DriveSimulatorApp;
import com.example.simulator.sim.Robot;
import com.example.simulator.sim.RobotController;
import com.example.simulator.sim.RobotState;
import com.example.simulator.sim.Telemetry;

public class SimTest {
    public static void main(String[] args) {
        final Bezier bezier = PurePursuit.builder
                .addControlPoint(0, 0)
                .addControlPoint(104.5, 103.5)
                .addControlPoint(107.5, -92)
                .addControlPoint(26.5, -36)
                .addControlPoint(-35.5, 29.5)
                .addControlPoint(140, 54.5)
                .addControlPoint(-37.5, -40)
                .build();
        RobotController controller = dt -> {
            ParametricPose targetPose = PurePursuit.INSTANCE.calculateTargetPose(bezier);
            RobotState.setVelocities(PurePursuit.INSTANCE.singlePIDtoPoint(targetPose.toPose()));
            Robot.targetPoint = targetPose.toPose();

//                RobotState.setVelocities(PurePursuit.INSTANCE.singlePIDtoPoint(bezier.end().toPose()));
        };

        Robot bot = new Robot(0,0, -Math.PI /2, controller);
        bot.addBezier(bezier);

        DriveSimulatorApp.setRobot(bot);
        DriveSimulatorApp.telemetry.addData("targetX", () -> PurePursuit.targetPose.x);
        DriveSimulatorApp.telemetry.addData("targetY", () -> PurePursuit.targetPose.y);
        DriveSimulatorApp.telemetry.addData("targetH", () -> PurePursuit.targetPose.h);
        DriveSimulatorApp.telemetry.addData("rotX", () -> PurePursuit.rotX);
        DriveSimulatorApp.telemetry.addData("rotY", () -> PurePursuit.rotY);
        DriveSimulatorApp.telemetry.addData("powH", () -> PurePursuit.powH);
        DriveSimulatorApp.telemetry.addData("numTargets", () -> PurePursuit.targetList.size());
        DriveSimulatorApp.main(args);
    }
}