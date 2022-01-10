package org.firstinspires.ftc.teamcode.Deprecated;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

@Deprecated
public class ObstacleAvoidanceV1 extends AbState {
    private HardwareHandler hardwareHandler;
    private double prevLeft, prevRight;
    private final double MinDistance = 1, SideDistance = 0.1, Speed = 0.3; // in units meters
    private boolean inFront, onTheSides;
    public ObstacleAvoidanceV1(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (inFront || onTheSides) { // run must be run before this
            return this;
        }
        return nextStateMap.get("next");
    }

    @Override
    public void run() {
        double[] obstacles = hardwareHandler.getSensorBoolean(); // we could have the left and right sensors at right angles to forward
        double left = obstacles[0], right = obstacles[obstacles.length-1];
        boolean incrLeft = left-prevLeft > 0 && left < SideDistance, incrRight = right-prevRight > 0 && right < SideDistance;

        onTheSides = incrLeft || incrRight;
        for (int i = 1; i < obstacles.length-1; i++) { // there are probably going to be three front sensors
            if (obstacles[i] < MinDistance) {
                inFront = true;
                break;
            }
        }

        if (incrLeft && incrRight) { // think about the order of this
            hardwareHandler.move(-1, 0, 0, Speed); // go back if we're going into a corner
        }
        else if (incrLeft) {
            hardwareHandler.move(0, -1, 0, Speed); // turn right if we're going into a wall on the left
        }
        else if (incrRight) {
            hardwareHandler.move(0, 1, 0, Speed); // turn left if we're going into a wall on the right
        }
        else { // prefers to avoid moving into a wall on the sides than in front
            if (left > right) { // if the path is more open on the left, turn towards there until it'ss open
                hardwareHandler.move(0, 1, 0, Speed); // prefers left
            } else { // if the path is more open on the right
                hardwareHandler.move(0, -1, 0, Speed);
            }
        }
        prevLeft = left;
        prevRight = right;
    }
}