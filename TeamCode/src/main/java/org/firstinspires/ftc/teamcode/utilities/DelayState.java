package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;

import java.util.HashMap;

public class DelayState extends AbState {
    private double time;
    private ElapsedTime timer;
    public DelayState(String name, double timeMSs) {
        super(name, "next");
        time = timeMSs;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (timer.milliseconds() > time) return nextStateMap.get("next");
        return this;
    }

    @Override
    public void run() {
        // do nothing
    }
}
