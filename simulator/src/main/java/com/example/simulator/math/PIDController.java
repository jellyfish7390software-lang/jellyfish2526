package com.example.simulator.math;

import org.jetbrains.annotations.NotNull;

public class PIDController extends PIDFController {

    /**
     * Default constructor with just the coefficients
     */
    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd, 0);
    }

    /**
     * The extended constructor.
     */
    public PIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, 0, sp, pv);
    }

    public void setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
    }

    public void setPID(PIDCoefficients coeffs) {
        setPID(coeffs.p, coeffs.i, coeffs.d);
    }
}
