package com.example.simulator.sim;

public class MotorPowers {
    public double frontLeft, backLeft, frontRight, backRight;

    public MotorPowers(double frontLeft,double backLeft,double frontRight,double backRight) {
        this.frontLeft = clamp(frontLeft, -1, 1);
        this.backLeft = clamp(backLeft, -1, 1);
        this.frontRight = clamp(frontRight, -1, 1);
        this.backRight = clamp(backRight, -1, 1);
    }


    public double clamp(double input, double leftBound, double rightBound) {
        if (leftBound < rightBound) {
            if (input < leftBound) input = leftBound;
            else if (input > rightBound) input = rightBound;

            return input;
        }
        return input;
    }
}
