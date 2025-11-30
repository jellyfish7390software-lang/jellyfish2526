package com.example.simulator.sim;

public class MecanumKinematics {
    public double gearRatio, rpm, wheelRadius, length, width;
    public double r;

    public MecanumKinematics(double gearRatio, double rpm, double wheelRadius, double length, double width) {
        this.gearRatio = gearRatio;
        this.rpm = rpm;
        this.wheelRadius = wheelRadius;
        this.length = length;
        this.width = width;
        r = Math.hypot(length / 2.0, width / 2.0);
    }

    public double powerToInchesPerSec(double power) {
        double wheelRPM = power * (rpm / gearRatio);
        double wheelCircumference = 2 * Math.PI * wheelRadius;
        return (wheelRPM / 60.0) * wheelCircumference;
    }

    public RobotVelocity compute(MotorPowers powers) {
        double vxPower = (powers.frontLeft + powers.frontRight + powers.backLeft + powers.backRight) / 4.0;
        double vyPower = (-powers.frontLeft + powers.frontRight + powers.backLeft - powers.backRight) / 4.0;
        double omegaPower = (-powers.frontLeft + powers.frontRight - powers.backLeft + powers.backRight) / 4.0;

        double vx = powerToInchesPerSec(vxPower);
        double vy = powerToInchesPerSec(vyPower);
        double omega = powerToInchesPerSec(omegaPower) / r; // radians/sec

        return new RobotVelocity(vx, vy, omega);
    }
}