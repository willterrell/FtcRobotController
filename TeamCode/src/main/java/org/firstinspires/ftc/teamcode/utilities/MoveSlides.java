package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.WaitForMotor;

import java.util.HashMap;

public class MoveSlides extends AbState {
    private HardwareHandler hardwareHandler;
    private double in;
    private WaitForMotor waitState;
    public MoveSlides(String name, HardwareHandler hardwareHandler, double in) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.in = in;
    }

    @Override
    public void init() {
        hardwareHandler.moveSlides(in);
        waitState = new WaitForMotor("Waiting for Slides: " + in, hardwareHandler.getSlideMotor());
        waitState.putNextState("next", getNextState("next"));
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return waitState;
    }

    @Override
    public void run() {

    }
}
