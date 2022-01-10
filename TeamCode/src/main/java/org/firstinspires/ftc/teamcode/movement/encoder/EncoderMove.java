package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PosType;

import java.util.HashMap;

public class EncoderMove extends AbState {
    private HardwareHandler hardwareHandler;
    private Position pos;
    private double angle;
    private AbState currState;
    private  double speed;
    private PosType type;
    public EncoderMove(String name, HardwareHandler hardwareHandler, Position pos, double angle, double speed, PosType posType) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.pos = pos;
        this.angle = angle;
        this.speed = speed;
        type = posType;
    }

    public double distance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // pythagorean theorem
    }

    @Override
    public void init() { // remember: angle -= angleTo;
        if (type == PosType.ABSOLUTE) {
            Position curr = hardwareHandler.getEncoderPosition();
            pos = new Position(DistanceUnit.METER, pos.x - curr.x, pos.y - curr.y, pos.z - curr.z, 0);
            angle = hardwareHandler.getEncoderAngle() - angle;
            hardwareHandler.addEncoderPosition(pos);
            hardwareHandler.addEncoderAngle(angle);
        }
        else {
            hardwareHandler.addEncoderPosition(pos);
            hardwareHandler.addEncoderAngle(angle);
        }

        double angleTo = Math.atan(pos.y/ pos.x);
        double distanceTo = distance(pos.x, pos.y);
        angle -= angleTo;

        EncoderRotateState rotateTo = new EncoderRotateState("RotateTo", angleTo, hardwareHandler, speed);
        EncoderForwardState moveTo = new EncoderForwardState("MoveTo", hardwareHandler, distanceTo, speed);
        EncoderRotateState finalRotate = new EncoderRotateState("FinalRotate", angle, hardwareHandler, speed);

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
