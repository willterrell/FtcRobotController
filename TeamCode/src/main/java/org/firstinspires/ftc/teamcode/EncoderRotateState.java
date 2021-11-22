package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class EncoderRotateState extends AbState {
    private double degrees;
    private HardwareHandler hardwareHandler;
    public EncoderRotateState(String name, double degrees, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.degrees = degrees;
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
        hardwareHandler.rotateWithEncoders(degrees);
    }
}
