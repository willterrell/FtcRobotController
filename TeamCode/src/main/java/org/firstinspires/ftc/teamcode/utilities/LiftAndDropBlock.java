package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class LiftAndDropBlock extends AbState {
    public LiftAndDropBlock(String name, HardwareHandler hardwareHandler, DropOffPosition pos_) {
        super(name, "next");
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
