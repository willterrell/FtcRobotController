package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD, p, i, d, prevError;
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getInput(double error, double dt) {
        if (p == 0 && i == 0 && d == 0 && prevError == 0) {
            p = error;
            i = error;
            d = 0;
            prevError = error;
            return 0;
        }
        p = error;
        i = (error + i) * dt;
        d = (error - prevError) / dt;
        return p * kP + i * kI + d * kD;
    }
}
