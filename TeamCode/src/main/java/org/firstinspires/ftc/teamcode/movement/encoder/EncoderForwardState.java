package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class EncoderForwardState extends AbState {
    private double meters;
    private HardwareHandler hardwareHandler;
    private double speed;
    public EncoderForwardState(String name, HardwareHandler hardwareHandler, double meters, double speed) {
        super(name);
        this.meters = meters;
        this.hardwareHandler = hardwareHandler;
        this.speed = speed;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return new EncoderWaitState("Wait for forward", hardwareHandler);
    }

    @Override
    public void run() {
        hardwareHandler.forwardWithEncoders(meters);
        hardwareHandler.setPowers(Math.signum(meters) * speed, Math.signum(meters) * speed, Math.signum(meters) * speed, Math.signum(meters) * speed);
    }
}
