package org.firstinspires.ftc.teamcode.barcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.StartPos;
import org.firstinspires.ftc.teamcode.utilities.ExpelBlock;

import java.util.HashMap;

public class DetectBlocksAndDropOff extends AbState {
    private HardwareHandler hardwareHandler;
    public DetectBlocksAndDropOff(String name, HardwareHandler hardwareHandler, StartPos side) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {
        EncoderMove moveToBarcode = null;

        MoveWithSensor moveFirstBlock = null;
        MoveWithSensor moveSecondBlock = null;
        MoveWithSensor moveThirdBlock = null;

        EncoderMove moveToHub = null;

        ExpelBlock firstTierExpel = null;
        ExpelBlock secondTierExpel = null;
        ExpelBlock thirdTierExpel = null;
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return null;
    }

    @Override
    public void run() {

    }
}
