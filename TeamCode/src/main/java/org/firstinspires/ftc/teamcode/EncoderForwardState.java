package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

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
        hardwareHandler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareHandler.forwardWithEncoders(meters);
        hardwareHandler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardwareHandler.setPowers(Math.signum(meters) * speed, Math.signum(meters) * speed, Math.signum(meters) * speed, Math.signum(meters) * speed);
    }
}
