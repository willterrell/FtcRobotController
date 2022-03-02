package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.DropOffPosition;

import java.util.HashMap;

public class LiftAndDropBlock extends AbState { // TODO implement this; ls height will be especially important
    private HardwareHandler hardwareHandler;
    public LiftAndDropBlock(String name, HardwareHandler hardwareHandler, DropOffPosition pos) {
        super(name, "next");
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

    }
}
