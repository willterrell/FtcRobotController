package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class temp extends AbState {
    private HardwareHandler hardwareHandler;
    public temp(String name, HardwareHandler hardwareHandler) {
        super(name, "nextStateName");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return null;

    }

    @Override
    public void run() {
        double[] sensorReading = hardwareHandler.getPylonSensorReadings();
        if (sensorReading[0] > 100 && sensorReading[1] > 100) {
            hardwareHandler.moveWithPower(1, 0, 0, 0.1);
        }
        else if (sensorReading[0] > sensorReading[1]) {
            hardwareHandler.moveWithPower(0, 0, 1, 0.1);
        }
        else if (sensorReading[0] < sensorReading[1]) {
            hardwareHandler.moveWithPower(0, 0, -1, 0.1);
        }
    }
}
