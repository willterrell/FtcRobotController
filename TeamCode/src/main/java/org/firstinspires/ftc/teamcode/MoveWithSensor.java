package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class MoveWithSensor extends AbState {
    private HardwareHandler hardwareHandler;
    public MoveWithSensor(String name, HardwareHandler hardwareHandler) {
        super(name);
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
        
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.getBlockSensorDetection()) {
            return this;
        }
        return null;
    }

    @Override
    public void run() {

    }
}
