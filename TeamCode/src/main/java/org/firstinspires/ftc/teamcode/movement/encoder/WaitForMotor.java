package org.firstinspires.ftc.teamcode.movement.encoder;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbState;

import java.util.HashMap;

public class WaitForMotor extends AbState {
    private DcMotor motor;
    public WaitForMotor(String name, DcMotor motor) {
        super(name, "next");
        this.motor = motor;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (motor.isBusy()) {
            return this;
        }
        return getNextState("next");
    }

    @Override
    public void run() {

    }
}
