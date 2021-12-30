package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public class EncoderRotateState extends AbState {
    private double degrees;
    private HardwareHandler hardwareHandler;
    private double speed;
    public EncoderRotateState(String name, double degrees, HardwareHandler hardwareHandler, double speed) {
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
        return nextStateMap.get("next");
    }

    @Override
    public void run() {
        hardwareHandler.rotateWithEncoders(degrees);
        hardwareHandler.setPowers(Math.signum(degrees) * -1 * speed, Math.signum(degrees) * -1 * speed, Math.signum(degrees) * speed, Math.signum(degrees) * speed);
    }
}
