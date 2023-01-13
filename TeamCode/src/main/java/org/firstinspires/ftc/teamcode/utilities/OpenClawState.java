package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class OpenClawState extends AbState {
    private HardwareHandler hardwareHandler;
    public OpenClawState(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return getNextState("next");
    }

    @Override
    public void run() {
        hardwareHandler.openClaw();
    }
}