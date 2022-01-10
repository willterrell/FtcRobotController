package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class ExpelBlock extends AbState {
    public ExpelBlock(String name, HardwareHandler hardwareHandler) {
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
