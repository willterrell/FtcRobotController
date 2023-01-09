package org.firstinspires.ftc.teamcode.movement.encoder;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

public class SlideWaitState extends AbState {
    private HardwareHandler hardwareHandler;
    private ElapsedTime timer;
    private TelemetryObj targetTele;
    public SlideWaitState(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        targetTele = new TelemetryObj("Target distance:", hardwareHandler.getTargetDistance());
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        timer.reset();
        addTele(targetTele);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.slideBusy()) {
            return this;
        }
        hardwareHandler.haltSlides();
        return nextStateMap.get("next");
    }

    @Override
    public void run() {

    }
}
