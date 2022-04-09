package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.ExpelOrInput;

import java.util.HashMap;

public class ExpelBlock extends AbState {
    private HardwareHandler hardwareHandler;
    private ElapsedTime timer;
    private ExpelOrInput type;
    private double runtimeMS;
    public ExpelBlock(String name, HardwareHandler hardwareHandler, ExpelOrInput type, double runtimeMS) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.type = type;
        this.runtimeMS = runtimeMS;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (timer.milliseconds() >= runtimeMS) {
            hardwareHandler.moveInputWheel(0);
            return nextStateMap.get("next");
        }
        return this;
    }

    @Override
    public void run() {
        if (type == ExpelOrInput.INPUT) hardwareHandler.moveInputWheel(1); // check powers
        else hardwareHandler.moveInputWheel(-1);
    }
}
