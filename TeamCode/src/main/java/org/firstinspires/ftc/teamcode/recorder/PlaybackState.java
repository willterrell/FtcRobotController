package org.firstinspires.ftc.teamcode.recorder;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.ArrayList;
import java.util.HashMap;

public class PlaybackState extends AbState { // TODO Deserialization
    private RecorderLog log;
    private double time = 0;
    private ElapsedTime timer;
    private double ticksPerSec;
    private double secondsPerTick;
    private double lastTime;

    public PlaybackState(String name, HardwareHandler hardwareHandler, RecorderLog log, double ticksPerSecond) {
        super(name, "next");
        this.log = log;
        this.ticksPerSec = ticksPerSecond;
        secondsPerTick = 1000/ticksPerSecond;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        lastTime = log.findLastTime();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (timer.milliseconds() > lastTime) return nextStateMap.get("next");
        return this;
    }

    @Override
    public void run() {
        if (timer.milliseconds() >= time) { // if statement ensures a set tps
            ArrayList<Runnable> actions;
            try {
                actions = log.getActionsWithParameters(time);
            }
            catch (ActionNotIncluded e) {
                actions = new ArrayList<>();
                // add error logging
            }
            for (Runnable action : actions) {
                action.run();
            }
            time = Math.round(timer.milliseconds());
            time += secondsPerTick - time % secondsPerTick; // rounds up the time
        }
    }
}
