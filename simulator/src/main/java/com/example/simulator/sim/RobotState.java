package com.example.simulator.sim;

import com.example.simulator.math.Pose;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
    public static double x, y;          // position in pixels
    public static double heading;       // radians
    public static double velocityX;     // pixels/sec
    public static double velocityY;     // pixels/sec
    public static double angularVelocity; // radians/sec

    public static void update(double dt) {
        double vx_global = velocityX * Math.cos(heading) - velocityY * Math.sin(heading);
        double vy_global = velocityX * Math.sin(heading) + velocityY * Math.cos(heading);

        x += (vx_global * dt);
        y += (vy_global * dt);
        heading += angularVelocity * dt;
        heading = DriveSimulatorApp.wrapAngle(heading);
    }

    public static void setVelocities(RobotVelocity vels) {
        velocityX = vels.vx;
        velocityY = vels.vy;
        angularVelocity = vels.omega;
    }

    public static Pose getPose() {
        return new Pose(x,y,heading);
    }

    public static List<Double> setDrivePowers(double x, double y, double rx) {
        List<Double> powerList = new ArrayList<>();

        // Adjusted for: +x = forward, +y = right, +rx = clockwise
        double frontLeft = x - y + rx;
        double backLeft = x + y + rx;
        double frontRight = x + y - rx;
        double backRight = x - y - rx;

        // Normalize powers to not exceed 1.0 magnitude
        double max = Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(backLeft)),
                Math.max(Math.abs(frontRight), Math.abs(backRight))
        );

        if (max > 1.0) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        powerList.add(frontLeft);
        powerList.add(backLeft);
        powerList.add(frontRight);
        powerList.add(backRight);

        return powerList;
    }

}
