package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderWaitState;
import org.firstinspires.ftc.teamcode.movement.encoder.SlideWaitState;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class MoveSlidesState extends AbState {
    private SlidePosition slidePosition;
    private HardwareHandler hardwareHandler;
    public MoveSlidesState(String name, HardwareHandler hardwareHandler, SlidePosition slidePosition) {
        super(name, "next");
        this.slidePosition = slidePosition;
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
        hardwareHandler.moveSlidesToPreset(slidePosition);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        AbState next = getNextState("next");
        SlideWaitState wait = new SlideWaitState("Wait", hardwareHandler);
        wait.putNextState("next", next);
        return wait;
    }

    @Override
    public void run() {

    }
}
