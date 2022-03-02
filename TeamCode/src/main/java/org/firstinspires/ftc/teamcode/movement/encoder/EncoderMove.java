package org.firstinspires.ftc.teamcode.movement.encoder;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

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
            double currAngle = hardwareHandler.getIMUZAngle();
            hardwareHandler.setEncoderPosition(pos);
            pos = hardwareHandler.normalize(pos, curr, currAngle); // check this
            angle = angle - currAngle;

        }
        else {
            hardwareHandler.addEncoderPosition(pos);
        }

        double angleTo = (pos.y != 0) ? Math.toDegrees(Math.atan(pos.x/pos.y)) : (pos.x > 0) ? 90 : -90; // y is forward
        if (pos.x == 0 && pos.y == 0) angleTo = 0;
        /*if (pos.x < 0) angleTo += 180; // makes sure atan covers 2nd and 3rd quadrants
        if (angleTo > 0) angleTo += 360; // makes sure angle is positive
        if (angleTo > 180) angleTo -= 360; // gets best direction for turning*/
        double distanceTo = distance(pos.x, pos.y);
        /*if (Math.abs(angleTo) > 90) { // optimizes turning with backwards movement
            distanceTo *= -1;
            angleTo = 90 * Math.signum(angleTo) - angleTo;
        }*/
        if (pos.y < 0) {
            distanceTo *= -1;
        }
        angle -= angleTo;

        RotateWithIMU rotateTo = new RotateWithIMU("RotateTo", hardwareHandler, angleTo, speed);
        EncoderForwardState moveTo = new EncoderForwardState("MoveTo", hardwareHandler, distanceTo, speed);
        RotateWithIMU finalRotate = new RotateWithIMU("FinalRotate", hardwareHandler, angle, speed);

        rotateTo.putNextState("next", moveTo);
        moveTo.putNextState("next", finalRotate);
        finalRotate.putNextState("next", super.getNextState("next"));
        currState = rotateTo;
        rotateTo.init();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return currState;
    }

    @Override
    public void run() {

    }
}
