package org.firstinspires.ftc.teamcode.structures;

public class PIDController {
    private double kP, kI, kD, p, i, d, c, prevError, iD, kC = 0, minError = 0;
    private boolean initialized = false;
    private boolean dampened = false;
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public PIDController(double kP, double kI, double kD, double iD) {
        this(kP, kI, kD);
        this.iD = Math.abs(iD); // iD dampens the I term (I will not grow beyond iD; prevents a build up of I basically)
        dampened = true;
    }
    public PIDController(double kP, double kI, double kD, double iD, double kC, double minError) {
        this(kP, kI, kD, iD);
        this.kC = kC;
        this.minError = minError;
    }

    public double getInput(double error) {
        if (!initialized) {
            p = kP * error;
            i = kI * error;
            d = 0;
            prevError = error;
            initialized = true;
            return 0;
        }
        p = kP * error;
        i = kI * error + i;
        if (dampened) i = Math.max(Math.min(i, iD), -iD); // clips i
        d = -kD * (error - prevError);
        prevError = error;
        c = (Math.abs(error) >= minError) ? Math.signum(error) * kC : 0;
        return p + i + d + c;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }
}