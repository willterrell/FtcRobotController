package org.firstinspires.ftc.teamcode.movement.imu;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PosType;

import java.util.HashMap;

public class MoveWithIMUState extends AbState {
    private HardwareHandler hardwareHandler;
    private Position target;
    private Position curr;
    private double targetAngle;
    private double currAngle;

    private final double DPrecision = 0.1; // how close the robot must get to the target before stopping
    private final double RPrecision = 15;
    // consider unit for these: meters, degrees
    private final double Speed;

    private PosType posType;


    public MoveWithIMUState(String name, HardwareHandler hardwareHandler, Position target, double targetAngle, double speed, PosType posType) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.target = target;
        this.targetAngle = targetAngle;
        this.posType = posType;
        this.Speed = speed;
    }

    @Override
    public void init() { // for the first next (won't matter usually but just in case)
        curr = hardwareHandler.getIMUPosition();
        currAngle = hardwareHandler.getIMUZAngle();
        if (posType == PosType.RELATIVE) {
            target = hardwareHandler.addPositions(target, curr);
            targetAngle += currAngle;
        }
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // pythagorean theorem
    }
    public double distance(Position reference, Position target) {
        return distance(target.x-reference.x, target.y-reference.y);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        Position diff = hardwareHandler.findRelativeDifference(curr, target, currAngle);
        if (distance(diff.x, diff.y) < DPrecision && Math.abs(targetAngle - currAngle) < RPrecision) { // we're not caring for rotation rigth now
            return nextStateMap.get("next"); // user must use this key
        }
        return this;
    }

    @Override
    public void run() {
        curr = hardwareHandler.getIMUPosition();
        currAngle = hardwareHandler.getIMUZAngle();
        Position diff = hardwareHandler.findRelativeDifference(curr, target, currAngle);
        double angleToTarget = 0;
        if (diff.x != 0) {
            angleToTarget = Math.atan(diff.y/diff.x);
        }
        if (Math.abs(angleToTarget) > RPrecision) { // if the robot is not facing the target
            hardwareHandler.moveWithPower(0, Math.signum(angleToTarget), 0, Speed);
        }
        else if (diff.y > DPrecision) { // once we're facing in the right direction, move towards target
            hardwareHandler.moveWithPower(Math.signum(diff.y), 0, 0, Speed);
        }
        else if (currAngle != targetAngle) { // once we're at the spot we need to be, orient ourselves to the direction of target
            hardwareHandler.moveWithPower(0, Math.signum(targetAngle - currAngle), 0, Speed);
        }
    }
}
