package org.firstinspires.ftc.teamcode.structures;

public class PIDController {
    private double kP, kI, kD, p, i, d, prevError;
    private boolean initialized = false;
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getInput(double error) {
        if (!initialized) {
            p = error;
            i = error;
            d = 0;
            prevError = error;
            initialized = true;
            return 0;
        }
        p = error;
        i = error + i;
        d = (error - prevError);
        return p * kP + i * kI + d * kD;
    }
}
