package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class EncoderStrafe extends AbState {
    private HardwareHandler hardwareHandler;
    private double distance, speed;
    public EncoderStrafe(String name, HardwareHandler hardwareHandler, double inches, double speed) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.distance = inches;
        this.speed = speed;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        EncoderWaitState wait = new EncoderWaitState("Wait for forward", hardwareHandler.getMotors());
        wait.putNextState("next", nextStateMap.get("next"));
        return wait;
    }

    @Override
    public void run() {
        hardwareHandler.strafeWithEncoders(distance, speed);
    }
}
