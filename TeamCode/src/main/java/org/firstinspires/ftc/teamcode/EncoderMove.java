package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.HashMap;

public class EncoderMove extends AbState {
    private HardwareHandler hardwareHandler;
    private Position pos;
    private double angle;
    private AbState currState;
    public EncoderMove(String name, HardwareHandler hardwareHandler, Position pos, double angle) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.pos = pos;
        this.angle = angle;
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // pythagorean theorem
    }

    @Override
    public void init() { // remember: angle -= angleTo;
        double angleTo = Math.atan(pos.y/ pos.x);
        double distanceTo = distance(pos.x, pos.y);
        angle -= angleTo;
        EncoderRotateState rotateTo = new EncoderRotateState("RotateTo", angleTo, hardwareHandler);
        EncoderForwardState moveTo = new EncoderForwardState("MoveTo", distanceTo, hardwareHandler);
        EncoderRotateState finalRotate = new EncoderRotateState("FinalRotate", angle, hardwareHandler);
        rotateTo.putNextState("next", moveTo);
        moveTo.putNextState("next", finalRotate);
        finalRotate.putNextState("next", super.getNextState("next"));
        currState = rotateTo;
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return currState;
    }

    @Override
    public void run() {

    }
}
