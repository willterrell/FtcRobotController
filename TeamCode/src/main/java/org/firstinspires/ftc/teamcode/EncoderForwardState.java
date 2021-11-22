package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class EncoderForwardState extends AbState {
    private double meters;
    private HardwareHandler hardwareHandler;
    public EncoderForwardState(String name, double meters, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.meters = meters;
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return nextStateMap.get("next");
    }

    @Override
    public void run() {
        hardwareHandler.forwardWithEncoders(meters);
    }
}
