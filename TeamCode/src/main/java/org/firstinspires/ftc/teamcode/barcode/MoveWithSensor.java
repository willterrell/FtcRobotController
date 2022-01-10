package org.firstinspires.ftc.teamcode.barcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.HashMap;

public class MoveWithSensor extends AbState { // moves to position, branches if a block is detected
    private HardwareHandler hardwareHandler;
    private AbState currState;
    private Position target;
    private double angle;
    private double speed;
    public MoveWithSensor(String name, HardwareHandler hardwareHandler, Position target, double angle, double speed) {
        super(name, "ifSensor", "ifNot");
        this.hardwareHandler = hardwareHandler;
        this.target = target;
        this.angle = angle;
        this.speed = speed;
    }

    @Override
    public void init() {
        EncoderMove move = new EncoderMove("SensorMove", hardwareHandler, target, angle, speed);
        PlaceholderState placeholder = new PlaceholderState();
        move.putNextState("next", placeholder);
        currState = move;
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.getBlockSensorDetection()) {
            return nextStateMap.get("ifSensor");
        }
        if (currState instanceof PlaceholderState) {
            return nextStateMap.get("ifNot");
        }
        return this;
    }

    @Override
    public void run() {
        currState.run();
        currState = currState.next();
    }
}
