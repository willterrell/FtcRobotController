package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.HashMap;

public class HardwareHandler {
    private HardwareMap hardwareMap;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private BNO055IMU imu;
    private SimpsonIntegrator integrator;
    private final int msPollInterval = 100;

    private final double wheelDiameter = 0.1; // in meters
    private final double width = 0.31;
    private final double length = 0.19;
    private final int ticksPerRotation = 7200;

    public HardwareHandler(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // Motor initiations here
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // imu shit here, supposedly we need to calibrate it
        integrator = new SimpsonIntegrator(msPollInterval);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = integrator;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //assert(imu.isSystemCalibrated()): "Calibrate the IMU";
    }

    public void initIMU(Position currPosition, Velocity currVelocity) { // should be called on the start of the opmode
        imu.startAccelerationIntegration(currPosition, currVelocity, msPollInterval); // example had it with 1000ms?
    }

    public void runRamp() { // activates the ramp at a constant speed
        /*...*/
    }

    public void runVacuum() { // activates vacuum
        /*...*/
    }

    public Position normalize(Position reference, Position transform, double angle) { // converts a transform from the perspective of angle
        if (!reference.unit.equals(transform.unit)) { // convert to same units
            reference = reference.toUnit(transform.unit);
        }
        Position diff = new Position(reference.unit, transform.x-reference.x, transform.y-reference.y, transform.x-reference.z, 0);
        diff = rotateTransform(diff, 90 - angle);
        return diff;
    }

    public Position rotateTransform(Position transform, double angle) { // rotates a 2d vector by angle
        double x = transform.x * Math.cos(angle) - transform.y * Math.sin(angle);
        double y = transform.x * Math.sin(angle) + transform.y * Math.cos(angle);
        return new Position(transform.unit, x, y, 0.0, 0); // TODO check this
    }

    public void moveWithEncoder(double d, double r) {
        /*...*/
    }

    public void autoTankMove(Position curr, double currAngle, Position target, double targetAngle) { // moves with tank drive; if we use tank tracks, remove strafe
        // this will work if move uses encoders
        // TODO convert this to a state
        if (!curr.unit.equals(target.unit)) { // convert to same units
            target = target.toUnit(curr.unit);
        }
        Position diff = normalize(curr, target, currAngle); // alter diff transform such that the robot is looking in the positive y direction
        double firstAngle = Math.atan(diff.y / diff.x);
        moveWithEncoder(0, firstAngle);
        double distance = Math.sqrt(Math.pow(diff.x, 2) + Math.pow(diff.y, 2));
        moveWithEncoder(distance, 0); // this doesn't work because the input should be power, not distance (FIX)
        double finalAngle = firstAngle + currAngle - targetAngle; // check this
        moveWithEncoder(0, finalAngle);
    }

    public void move(double d, double r, double speed) { // d : linear movement, r : rotational movement, s : speed (0-1); r is signed with CCW as positive
        assert (speed <= 1 && speed >= 0): "Speed must be between 0 and 1";
        // add motor type assertion or change
        double total = Math.abs(d) + Math.abs(r);
        if (d == 0 && r == 0) {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        leftFront.setPower((d+r)/total*speed);
        leftRear.setPower((d+r)/total*speed);
        rightFront.setPower((d-r)/total*speed);
        rightRear.setPower((d-r)/total*speed);
    }

    public double getIMUZAngle() { // gives current position as a double list formatted [x, y, r]
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public Position getIMUPosition() {
        Position pos = imu.getPosition();
        pos.toUnit(DistanceUnit.METER);
        return pos;
        // maybe do some transform on this so its oriented with y going forward and x sideways
    }

    public double[] getSensorBoolean() { // should return in order left-most to right-most
        return null;
    }

    public Acceleration getIMUAccel() {
        Acceleration accel = imu.getLinearAcceleration();
        accel.toUnit(DistanceUnit.METER);
        return accel;
    }

    public HashMap<String, Object> getIMUTelemetry() {
        return integrator.getTelemetry();
    }

    public boolean isBusy() {
        return leftFront.isBusy() ||
                rightFront.isBusy() ||
                leftRear.isBusy() ||
                rightRear.isBusy();
    }

    public void setMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftRear.setMode(mode);
        rightRear.setMode(mode);
    }

    public void setTargets(int tickLF, int tickRF, int tickLR, int tickRR) {
        leftFront.setTargetPosition(tickLF);
        rightFront.setTargetPosition(tickRF);
        leftRear.setTargetPosition(tickLR);
        rightRear.setTargetPosition(tickRR);
    }

    public void setForwardTargets(int ticks) {
        setTargets(ticks, ticks, ticks, ticks);
    }

    public void setRotateTargets(int ticks) { // CCW is +
        setTargets(ticks, -1 * ticks, ticks, -1 * ticks);
    }

    public void forwardWithEncoders(double distance) { // only needs to be run once
        int ticks = (int) (distance / ticksPerRotation * Math.PI * wheelDiameter);
        setForwardTargets(ticks);
    }

    public void rotateWithEncoders(double degrees) {
        double metersPerDegree = Math.PI * Math.sqrt(length*length + width*width) / 360;
        double metersPerTick = ticksPerRotation * Math.PI * wheelDiameter; // ticks * mpt / mpd = degrees
        int ticks = (int) (degrees / metersPerTick * metersPerDegree);
        setRotateTargets(ticks);
    }
}
