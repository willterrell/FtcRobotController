package org.firstinspires.ftc.teamcode.movement.imu;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.HashMap;

public class SimpsonIntegrator implements BNO055IMU.AccelerationIntegrator { // this will use simpson's 3/8 rule to approximate integration
    private ArrayList<Acceleration> accels;
    private ArrayList<Velocity> vels;
    private ArrayList<Position> poss; // we have to go back three integrations to get currPos
    private Position initPos;
    private Velocity initVel;
    private final double timeStep;

    private double simpsons38(double x0, double x1, double x2, double x3) {
        return (3 * timeStep / 8) * (x0 + 3 * x1 + 3 * x2 + x3); // this is Simpson's 3/8 rule: https://en.wikipedia.org/wiki/Simpson%27s_rule
    }

    private double simpsons13(double x0, double x1, double x2) {
        return (timeStep / 3) * (x0 + 4 * x1 + x2); // this is Simpson's 1/3 rule
    }

    private double trapezoid(double x0, double x1) {
        return (timeStep / 2) * (x0 + x1);
    }

    public SimpsonIntegrator(double msPollInterval) {
        timeStep = msPollInterval/1000; // 1000 to convert to seconds
    }
    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        accels = new ArrayList<>();
        vels = new ArrayList<>();
        poss = new ArrayList<>(); // throws null reference
        initPos = initialPosition;
        initVel = initialVelocity;
        if (initPos == null) {
            initPos = new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0);
        }
        if (initVel == null) {
            initVel = new Velocity(DistanceUnit.METER, 0.0, 0.0, 0.0, 0);
        }
        poss.add(initPos);
        vels.add(initVel);
    }

    @Override
    public Position getPosition() {
        return poss.get(poss.size()-1);
    }

    @Override
    public Velocity getVelocity() {
        return vels.get(vels.size()-1);
    }

    @Override
    public Acceleration getAcceleration() {
        if (accels.isEmpty()) return new Acceleration();
        return accels.get(accels.size()-1);
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        accels.add(linearAcceleration); // if we only have one accel reading, can't do anything
        if (accels.size() > 4) accels.remove(0);
        if (accels.size() == 2) { // if we have two accel readings, use trapezoid
            Acceleration a0 = accels.get(0), a1 = accels.get(1);
            double vx = trapezoid(a0.xAccel, a1.xAccel); // this approximates the integral from the time of a0 to a1
            double vy = trapezoid(a0.yAccel, a1.yAccel);
            double vz = trapezoid(a0.zAccel, a1.zAccel);
            Velocity v0 = vels.get(0), v1 = new Velocity(a0.unit, vx + v0.xVeloc, vy + v0.yVeloc, vz + v0.zVeloc, 0); // remember to add to v0
            vels.add(v1);
            double xx = trapezoid(v0.xVeloc, v1.xVeloc); // we have to double integrate
            double xy = trapezoid(v0.yVeloc, v1.yVeloc);
            double xz = trapezoid(v0.zVeloc, v1.zVeloc);
            Position x0 = poss.get(0), newPos = new Position(v0.unit, xx + x0.x, xy + x0.y, xz + x0.z, 0);
            poss.add(newPos);
        }
        else if (accels.size() == 3) { // there's definitely a better way to do this
            Acceleration a0 = accels.get(0), a1 = accels.get(1), a2 = accels.get(2);
            double vx = simpsons13(a0.xAccel, a1.xAccel, a2.xAccel);
            double vy = simpsons13(a0.yAccel, a1.yAccel, a2.yAccel);
            double vz = simpsons13(a0.zAccel, a1.zAccel, a2.zAccel);
            Velocity v0 = vels.get(0), v1 = vels.get(1), v2 = new Velocity(a0.unit, vx + v0.xVeloc, vy + v0.yVeloc, vz + v0.zVeloc, 0);
            vels.add(v2);
            double xx = simpsons13(v0.xVeloc, v1.xVeloc, v2.xVeloc);
            double xy = simpsons13(v0.yVeloc, v1.yVeloc, v2.yVeloc);
            double xz = simpsons13(v0.zVeloc, v1.zVeloc, v2.zVeloc);
            Position x0 = poss.get(0), newPos = new Position(v0.unit, xx + x0.x, xy + x0.y, xz + x0.z, 0);
            poss.add(newPos);
        }
        else if (accels.size() == 4) {
            Acceleration a0 = accels.get(0), a1 = accels.get(1), a2 = accels.get(2), a3 = accels.get(3);
            double vx = simpsons38(a0.xAccel, a1.xAccel, a2.xAccel, a3.xAccel);
            double vy = simpsons38(a0.yAccel, a1.yAccel, a2.yAccel, a3.yAccel);
            double vz = simpsons38(a0.zAccel, a1.zAccel, a2.zAccel, a3.zAccel);
            if (vels.size() == 4) vels.remove(0);
            Velocity v0 = vels.get(0), v1 = vels.get(1), v2 = vels.get(2), v3 = new Velocity(a0.unit, vx + v0.xVeloc, vy + v0.yVeloc, vz + v0.zVeloc, 0);
            vels.add(v3);
            double xx = simpsons38(v0.xVeloc, v1.xVeloc, v2.xVeloc, v3.xVeloc);
            double xy = simpsons38(v0.yVeloc, v1.yVeloc, v2.yVeloc, v3.yVeloc);
            double xz = simpsons38(v0.zVeloc, v1.zVeloc, v2.zVeloc, v3.zVeloc);
            if (poss.size() == 4) poss.remove(0);
            Position x0 = poss.get(0), newPos = new Position(v0.unit, xx + x0.x, xy + x0.y, xz + x0.z, 0);
            poss.add(newPos);
        }
    }

    public HashMap<String, Object> getTelemetry() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("accels", accels);
        map.put("vels", vels);
        map.put("poss", poss);
        return map;
    }
}
