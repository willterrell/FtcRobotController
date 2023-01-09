package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

public class EncoderForwardState extends AbState {
    private double inches;
    private HardwareHandler hardwareHandler;
    private double speed;
    private TelemetryObj targetDistance;
    public EncoderForwardState(String name, HardwareHandler hardwareHandler, double inches, double speed) {
        super(name, "next");
        this.inches = inches;
        this.hardwareHandler = hardwareHandler;
        this.speed = speed;
        targetDistance = new TelemetryObj("Target Distance: ", inches);
    }

    @Override
    public void init() {
        addTele(targetDistance);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        EncoderWaitState wait = new EncoderWaitState("Wait for forward", hardwareHandler);
        wait.putNextState("next", nextStateMap.get("next"));
        return wait;
    }

    @Override
    public void run() {
        hardwareHandler.goForwardWithEncoders(inches, speed);
    }
}
