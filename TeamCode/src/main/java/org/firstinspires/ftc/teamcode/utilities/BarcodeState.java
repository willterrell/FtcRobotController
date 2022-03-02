package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.DropOffPosition;

import java.util.HashMap;

public class BarcodeState extends AbState {
    private HardwareHandler hardwareHandler;
    private int barcodeNum = 0;
    private LiftAndDropBlock liftState;
    private double initAngle;
    private double input = 1;
    public BarcodeState(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
        initAngle = hardwareHandler.getIMUZAngle();
        initAngle = (initAngle < 0) ? initAngle + 360 : initAngle;
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (barcodeNum == 1) {
            LiftAndDropBlock lift = new LiftAndDropBlock("Bottom Tier Lift", hardwareHandler, DropOffPosition.BOTTOM_TIER);
            lift.putNextState("next", nextStateMap.get("next"));
        }
        if (barcodeNum == 2) {
            LiftAndDropBlock lift = new LiftAndDropBlock("Middle Tier Lift", hardwareHandler, DropOffPosition.MIDDLE_TIER);
            lift.putNextState("next", nextStateMap.get("next"));
        }
        if (barcodeNum == 3) {
            LiftAndDropBlock lift = new LiftAndDropBlock("Top Tier Lift", hardwareHandler, DropOffPosition.TOP_TIER);
            lift.putNextState("next", nextStateMap.get("next"));
        }
        return this;
    }

    @Override
    public void run() {
        double dis = hardwareHandler.getBarcodeDistance(DistanceUnit.INCH);
        double currAngle = hardwareHandler.getIMUZAngle();
        currAngle = (currAngle < 0) ? currAngle + 360 : currAngle;
        if (dis < 13) {
            barcodeNum = 1;
        }
        else if (dis < 21.5) {
            barcodeNum = 2;
        }
        else if (dis < 28) {
            barcodeNum = 3;
        }
        else { // rotates and searches for capstone
            double diff = currAngle - initAngle;
            input = (Math.abs(diff) > 20) ? -Math.signum(diff) : input;
            hardwareHandler.move(0, input, 0, 0.3);
        }
    }
}
