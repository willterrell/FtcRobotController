package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

public class RotateWithIMU extends AbState {
    private HardwareHandler hardwareHandler;
    private PIDController pid;
    private double targetAngle;
    private double currAngle;
    private ElapsedTime timer;
    private double prevTime;
    public RotateWithIMU(String name, HardwareHandler hardwareHandler, double angle) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        targetAngle = angle;
    }

    @Override
    public void init() {
        pid = new PIDController(1, 0.01, 0);
        currAngle = hardwareHandler.getIMUZAngle();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (Math.abs(currAngle - targetAngle) < 5) { // within 5 degrees of target
            return nextStateMap.get("next");
        }
        return this;
    }

    @Override
    public void run() {
        double ms = timer.milliseconds();
        if (prevTime == 0) prevTime = ms;
        currAngle = hardwareHandler.getIMUZAngle();
        double input = pid.getInput(targetAngle - currAngle);
        if (Math.abs(input) > 1) input = Math.signum(input); // clips input
        hardwareHandler.move(0, Math.signum(input), 0, 0.3);
        prevTime = ms;
    }
}
