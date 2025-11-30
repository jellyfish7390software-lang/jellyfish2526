package com.example.simulator.sim;

import com.example.simulator.math.Pose;
import com.example.simulator.math.PurePursuit;

import java.awt.BorderLayout;
import java.awt.Point;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

public class DriveSimulatorApp {
    public static final int MAX_TIME_MS = 60_000;
    public static Robot bot;
    public static final Telemetry telemetry = new Telemetry();

    public static void setRobot(Robot robot) {
        DriveSimulatorApp.bot = robot;
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            long startTime = System.currentTimeMillis();

            JFrame frame = new JFrame("Drive Simulator");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(1000, 800);
            frame.setLayout(new BorderLayout());

            FieldPanel fieldPanel = new FieldPanel(DriveSimulatorApp.bot);

            frame.add(fieldPanel, BorderLayout.CENTER);
            frame.add(telemetry, BorderLayout.EAST);

            JLabel timeLabel = new JLabel();
            timeLabel.setHorizontalAlignment(SwingConstants.CENTER);
            timeLabel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
            frame.add(timeLabel, BorderLayout.SOUTH);

            final long[] lastUpdateTime = { System.currentTimeMillis() };
            final long[] elapsedTime = { 0 };

            Timer updateTimer = new Timer(100, e -> {
                long now = System.currentTimeMillis();
                long delta = now - lastUpdateTime[0];
                lastUpdateTime[0] = now;

                if (!FieldPanel.paused) {
                    elapsedTime[0] += delta;
                }

                long displayTime = elapsedTime[0] % MAX_TIME_MS;
                timeLabel.setText("Time: " + formatTime(displayTime) + " / " + formatTime(MAX_TIME_MS));

                telemetry.updateAll();
            });
            updateTimer.start();

            telemetry.addData("X (in)", () -> RobotState.x);
            telemetry.addData("Y (in)", () -> RobotState.y);
            telemetry.addData("HeadingRad", () -> wrapAngle(RobotState.heading));
            telemetry.addData("HeadingDeg", () -> Math.toDegrees(wrapAngle(RobotState.heading)));
            telemetry.addData("X Velocity (in/s)", () -> RobotState.velocityX);
            telemetry.addData("Y Velocity (in/s)", () -> RobotState.velocityY);
            telemetry.addData("Angular Velocity", () -> RobotState.angularVelocity);
            telemetry.addData("Status", () -> FieldPanel.paused ? "Paused" : "Running");

            frame.setVisible(true);
        });
    }

    public static double wrapAngle(double angRad) {
        while (angRad >= 2*Math.PI) {
            angRad -= 2*Math.PI;
        }
        while (angRad < 0) {
            angRad += 2*Math.PI;
        }
        return angRad;
    }

    private static String formatTime(long timeMs) {
        long totalSeconds = timeMs / 1000;
        long minutes = totalSeconds / 60;
        long seconds = totalSeconds % 60;
        return String.format("%02d:%02d", minutes, seconds);
    }
}