package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class CloseClawState extends AbState {
    private HardwareHandler hardwareHandler;
    private ElapsedTime timer;
    private double startTime;
    public static double forTime = 500;
    public CloseClawState(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        startTime = timer.milliseconds();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (timer.milliseconds() - startTime > 100) return getNextState("next");
        return this;
    }

    @Override
    public void run() {
        hardwareHandler.closeClaw();
    }
}
