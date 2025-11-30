package org.firstinspires.ftc.teamcode.purepursuit.pid;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class SquIDController {
    public double kSQ ;

    public SquIDController(double kSQ) {
        this.kSQ = kSQ;
    }
    public SquIDController(PIDCoefficients pid) {
        kSQ = pid.p;
    }
    public void setSquID(double kSQ) {
        this.kSQ = kSQ;
    }
    public void setSquID(PIDCoefficients pid) {
        kSQ = pid.p;
    }

    public double compute(double error) {
        return kSQ * Math.pow(Math.abs(error), 1/1.5) * Math.signum(error);
    }
}
