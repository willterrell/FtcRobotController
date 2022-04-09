package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.DropOffPosition;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

public class BarcodeState extends AbState {
    private HardwareHandler hardwareHandler;
    private int barcodeNum = 0;
    private LiftAndDropBlock liftState;
    private double initAngle;
    private double input = 1, diff = 0;
    private Side side;
    private TelemetryObj rotationTele, dSensorTele, inputTele;
    private ElapsedTime timer;
    public BarcodeState(String name, HardwareHandler hardwareHandler, Side side) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.side = side;
        rotationTele = new TelemetryObj("Rotation");
        dSensorTele = new TelemetryObj("Sensed Distance");
        inputTele = new TelemetryObj("Rot Input");
    }

    @Override
    public void init() {
        initAngle = hardwareHandler.getIMUZAngle() % 360;
        timer = new ElapsedTime();
        addTele(rotationTele);
        addTele(dSensorTele);
        addTele(inputTele);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (barcodeNum != 0) {
            DropOffPosition dropOffPosition;
            if (barcodeNum == 1) dropOffPosition = DropOffPosition.BOTTOM_TIER;
            else if (barcodeNum == 2) dropOffPosition = DropOffPosition.MIDDLE_TIER;
            else dropOffPosition = DropOffPosition.TOP_TIER;
            RotateWithIMU rotate = new RotateWithIMU("Rotate to normal", hardwareHandler, -diff, 0.5);
            LiftAndDropBlock lift = new LiftAndDropBlock(dropOffPosition.toString() + " Lift", hardwareHandler, dropOffPosition, side);
            rotate.putNextState("next", lift);
            lift.putNextState("next", nextStateMap.get("next"));
            hardwareHandler.moveWithPower(0,0,0,0);
            return rotate;
        }

        return this;
    }

    @Override
    public void run() {
        double dis = hardwareHandler.getBarcodeDistance(DistanceUnit.INCH);
        double currAngle = hardwareHandler.getIMUZAngle() % 360;
        dSensorTele.setContent(dis);
        if (dis < 13) {
            barcodeNum = 1;
        }
        else if (dis < 21.5) {
            barcodeNum = 2;
        }
        else if (dis < 28 || timer.milliseconds() > 5000) {
            barcodeNum = 3;
        }
        else { // rotates and searches for capstone
            diff = (currAngle - initAngle) % 360;
            rotationTele.setContent(diff);
            input = (Math.abs(diff) > 20 && Math.signum(diff) == -Math.signum(input)) ? -input : input;
            inputTele.setContent(input);
            hardwareHandler.moveWithPower(0, input, 0, 0.3);
        }
    }
}
