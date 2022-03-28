package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class EncoderRotateState extends AbState {
    private double degrees;
    private HardwareHandler hardwareHandler;
    private double speed;

    public EncoderRotateState(String name, HardwareHandler hardwareHandler, double degrees, double speed) {
        super(name, "next");
        this.degrees = degrees;
        this.hardwareHandler = hardwareHandler;
        this.speed = speed;
    }

    @Override
    public void init() {
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        AbState next = nextStateMap.get("next");
        return next;
    }

    @Override
    public void run() {
        hardwareHandler.rotateWithEncoders(degrees);
        hardwareHandler.setDriveTrainPowers(speed, speed, speed, speed);
    }
}
