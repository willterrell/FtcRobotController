package org.firstinspires.ftc.teamcode.structures;

public class PIDFController {
    private double kP, kI, kD, kS, kG, p, i, d, prevError, iD, minError = 0;
    private boolean initialized = false;
    private boolean dampened = false;
    public PIDFController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public PIDFController(double kP, double kI, double kD, double iD) {
        this(kP, kI, kD);
        this.iD = Math.abs(iD); // iD dampens the I term (I will not grow beyond iD; prevents a build up of I basically)
        dampened = true;
    }

    public PIDFController(double kP, double kI, double kD, double kS, double kG, double minError) {
        this(kP, kI, kD);
        this.kS = kS;
        this.kG = kG;
        this.minError = minError;
    }

    public PIDFController(double kP, double kI, double kD, double iD, double kS, double kG, double minError) {
        this(kP, kI, kD, kS, kG, minError);
        this.iD = Math.abs(iD);
        dampened = true;
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
        double s = (Math.abs(error) >= minError) ? Math.signum(error) * kS : 0;
        return p + i + d + s + kG;
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

    public void setCoefficientsAndReset(double kP, double kI, double kD, double kS, double kG, double minError) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kG = kG;
        p = 0;
        i = 0;
        d = 0;
    }
}