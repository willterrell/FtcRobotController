package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class TankMoveState extends AbState {
    private AbState nextState;
    private HardwareHandler hardwareHandler;
    private Position target;
    private Position curr;
    private double targetAngle;
    private double currAngle;

    private final double DPrecision = 5; // how close the robot must get to the target before stopping
    private final double RPrecision = 5;
    // consider unit for these
    private final double Speed = 0.5;


    public TankMoveState(String name, AbState nextState, HardwareHandler hardwareHandler, Position target, double targetAngle) {
        super(name);
        this.nextState = nextState;
        this.hardwareHandler = hardwareHandler;
        this.target = target;
        this.targetAngle = targetAngle;
    }

    @Override
    public void init() {
        curr = hardwareHandler.getIMUPosition();
        currAngle = hardwareHandler.getIMUZAngle();
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // pythagorean theorem
    }
    public double distance(Position reference, Position target) {
        return distance(target.x-reference.x, target.y-reference.y);
    }

    @Override
    public AbState next() {
        Position diff = hardwareHandler.normalize(curr, target, currAngle);
        if (distance(diff.x, diff.y) < DPrecision) { // we're not caring for rotation rigth now
            return nextState;
        }
        return this;
    }

    @Override
    public void run() {

        curr = hardwareHandler.getIMUPosition();
        currAngle = hardwareHandler.getIMUZAngle();
        Position diff = hardwareHandler.normalize(curr, target, currAngle);
        if (Math.abs(diff.x) > RPrecision) { // if the robot is not facing the target
            hardwareHandler.move(0, Math.signum(diff.x), Speed);
        }
        else if (distance(diff.x, diff.y) > DPrecision) { // once we're facing in the right direction, move towards target
            hardwareHandler.move(Math.signum(diff.y), 0, Speed);
        }
        else if (currAngle != targetAngle) { // once we're at the spot we need to be, orient ourselves to the direction of target
            hardwareHandler.move(0, Math.signum(targetAngle - currAngle), Speed);
        }
    }
}
