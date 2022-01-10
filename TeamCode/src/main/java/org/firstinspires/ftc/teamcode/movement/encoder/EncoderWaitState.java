package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class EncoderWaitState extends AbState {
    private HardwareHandler hardwareHandler;
    public EncoderWaitState(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.isBusy()) {
            return this;
        }
        return nextStateMap.get("next");
    }

    @Override
    public void run() {

    }
}
