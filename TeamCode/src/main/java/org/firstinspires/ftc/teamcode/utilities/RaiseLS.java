package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;
import java.util.Locale;

public class RaiseLS extends AbState {
    private HardwareHandler hardwareHandler;
    private int lsPos;
    private TelemetryObj lsPositionTele, lsTargetPositionTele;
    public RaiseLS(String name, HardwareHandler hardwareHandler, int lsPos) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.lsPos = lsPos;
        lsPositionTele = new TelemetryObj("LS Pos");
        lsTargetPositionTele = new TelemetryObj("LS Target", lsPos);
    }

    @Override
    public void init() {
        hardwareHandler.setSlideSetpoint(lsPos);
        addTele(lsPositionTele);
        addTele(lsTargetPositionTele);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.areSlidesAtSetpoint(50, 25)){
            hardwareHandler.moveSlidesWithPower(0,0);
            return nextStateMap.get("next");
        }
        return this;
    }

    @Override
    public void run() {
        hardwareHandler.updateSlides();
        int[] curPos = hardwareHandler.getLSEncoderPosition();
        lsPositionTele.setContent(String.format(Locale.ENGLISH,"%d, %d", curPos[0], curPos[1]));
    }
}
