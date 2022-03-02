package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.movement.imu.SimpsonIntegrator;
import org.firstinspires.ftc.teamcode.structures.LSType;
import org.firstinspires.ftc.teamcode.structures.PIDController;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

public class HardwareHandler {
    private final HardwareMap hardwareMap;
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotor linearSlideLeft;
    private final DcMotor linearSlideRight;
    private final DcMotor carousel;
    private final DcMotor input;
    private final DistanceSensor barcode;
    private BNO055IMU imu;
    private SimpsonIntegrator integrator;
    private final int msPollInterval = 100;


    /*
    TODO
     change wheel diameter and robot length
     */
    private final double wheelDiameter = 4; // in inches
    private final double width = 0.31;
    private final double length = 0.19;
    private final int ticksPerRotation = 1120;
    private int lSTargetPos = 0;

    private PIDController upRLSPID, upLLSPID, diffLSPID;
    private boolean upRDis = true, upLDis = true, diffDis = true;
    private double upRKP = 0.0009562, upRKI = 0.000001, upRKD = 0, upLKP = 0.001660, upLKI = 0.000001, upLKD = 0, dKP = 0.001679, dKI = 0.000001, dKD = 0; // coefficients for linearSlidePID 1 and 2
    // it might just be better to have it only be a p controller since the target is changing

    private DcMotor.RunMode currRunMode;

    private Position currPos;
    private double currAngle;

    public HardwareHandler(HardwareMap hardwareMap, Position currPos) {
        this.hardwareMap = hardwareMap;
        // Motor initiations here
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");

        linearSlideLeft = hardwareMap.dcMotor.get("linearSlideLeft");
        linearSlideRight = hardwareMap.dcMotor.get("linearSlideRight");
        carousel = hardwareMap.dcMotor.get("carousel");
        input = hardwareMap.dcMotor.get("scoop");
        barcode = hardwareMap.get(DistanceSensor.class, "barcode");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        input.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        input.setDirection(DcMotorSimple.Direction.REVERSE);
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);

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
        
        upLLSPID = new PIDController(upLKP, upLKI, upLKD);
        upRLSPID = new PIDController(upRKP, upRKI, upRKD);
        diffLSPID = new PIDController(dKP, dKI, dKD);

        //assert(imu.isSystemCalibrated()): "Calibrate the IMU";

        this.currPos = currPos;
    }


    /*
                - DRIVETRAIN -
                  ~ TELEOP ~
     */


    public void move(double d, double r, double s, double speed) { // d : linear movement, r : rotational movement, s : speed (0-1); r is signed with CCW as positive
        //assert (speed <= 1 && speed >= 0): "Speed must be between 0 and 1";
        // add motor type assertion or change
        if (currRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed = Math.abs(speed);
        double total = Math.abs(d) + Math.abs(r);
        if (d == 0 && r == 0) {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        else {
            leftFront.setPower((d - r + s) / total * speed);
            leftRear.setPower((d - r - s) / total * speed);
            rightFront.setPower((d + r + s) / total * speed);
            rightRear.setPower((d + r - s) / total * speed);
        }
    }

    public double[] getVelocities() {
        return new double[]{leftFront.getVelocity(), leftRear.getVelocity(), rightFront.getVelocity(), rightRear.getVelocity()};
    }

    public double[] getPowers() {
        return new double[]{leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower()};
    }


    /*
                ~ AUTONOMOUS ~
     */


    /*public void autoTankMove(Position curr, double currAngle, Position target, double targetAngle) { // moves with tank drive; if we use tank tracks, remove strafe
        // this will work if move uses encoders
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
    */
    public Position getEncoderPosition() {
        return currPos;
    }

    public double[] getMotorPositions() {return new double[]{leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};}

    public void addEncoderPosition(Position pos) {
        double x = currPos.x + pos.x, y = currPos.y + pos.y;
        x = (Math.abs(x) > 70.125) ? 70.125 * Math.signum(x) : x; // clips position to the field
        y = (Math.abs(y) > 70.125) ? 70.125 * Math.signum(y) : y;
        currPos = addPositions(currPos, pos);
    }

    public double getEncoderAngle() {
        return currAngle;
    }

    public void addEncoderAngle(double angle) {
        currAngle += angle;
    }

    public void setEncoderPosition(Position pos) {
        currPos = pos;
    }


    public void setPowers(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
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
        currRunMode = mode;
    }

    public void setTargets(int tickLF, int tickRF, int tickLR, int tickRR) {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(-tickLF - leftFront.getCurrentPosition()); // 3.125 is for gearing
        rightFront.setTargetPosition(-tickRF - rightFront.getCurrentPosition());
        leftRear.setTargetPosition(-tickLR - leftRear.getCurrentPosition());
        rightRear.setTargetPosition(-tickRR - rightRear.getCurrentPosition());

        TelemetryFactory.add(new TelemetryObj<>("lf", leftFront.getCurrentPosition()));
        TelemetryFactory.add(new TelemetryObj<>("rf", leftRear.getCurrentPosition()));
        TelemetryFactory.add(new TelemetryObj<>("lr", leftRear.getCurrentPosition()));
        TelemetryFactory.add(new TelemetryObj<>("rr", rightRear.getCurrentPosition()));

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setForwardTargets(int ticks) {
        setTargets(ticks, ticks, ticks, ticks);
    }

    @Deprecated
    public void setRotateTargets(int ticks) { // CCW is +
        setTargets(ticks, -ticks, ticks, -ticks);
    }

    public void forwardWithEncoders(double distance) { // only needs to be run once
        int ticks = (int) (distance * ticksPerRotation / Math.PI / wheelDiameter * 48 / (50.5));
        setForwardTargets(ticks);
    }

    @Deprecated
    public void rotateWithEncoders(double degrees) {
        double inPerDegree = Math.PI * Math.sqrt(length*length + width*width) / 360;
        double inPerTick = ticksPerRotation * Math.PI * wheelDiameter; // ticks * mpt / mpd = degrees
        int ticks = (int) (degrees / inPerTick * inPerDegree);
        setRotateTargets(ticks);
    }

    public void stopDrivetrain() {
        setPowers(0, 0, 0, 0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTargetDistance() {
        return leftFront.getTargetPosition();
    }


    /*
                - IMU -
     */


    public void initIMU(Velocity currVelocity) { // should be called on the start of the opmode
        imu.startAccelerationIntegration(currPos, currVelocity, msPollInterval); // example had it with 1000ms?
    }

    public double getIMUZAngle() { // gives current position as a double list formatted [x, y, r]
        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return currAngle;
    }

    public Position getIMUPosition() {
        Position pos = imu.getPosition();
        currPos = pos;
        if (pos.unit != DistanceUnit.INCH) pos.toUnit(DistanceUnit.INCH);
        return pos;
        // maybe do some transform on this so its oriented with y going forward and x sideways
    }

    public Acceleration getIMUAccel() {
        Acceleration accel = imu.getLinearAcceleration();
        if (accel.unit != DistanceUnit.INCH) accel.toUnit(DistanceUnit.INCH);
        return accel;
    }

    public HashMap<String, Object> getIMUTelemetry() {
        return integrator.getTelemetry();
    }


    /*
                - LINEAR SLIDES -
     */


    public double[] updateSlides() {
        int leftPos = linearSlideLeft.getCurrentPosition();
        int rightPos = linearSlideRight.getCurrentPosition();
        int diff = leftPos - rightPos;

        double upLInput = upLLSPID.getInput(lSTargetPos - leftPos);
        double upRInput = upRLSPID.getInput(lSTargetPos - rightPos);
        double diffInput = diffLSPID.getInput(diff);

        double kUL = (upLDis ? 1 : 0);
        double kUR = (upRDis ? 1 : 0);
        double kDiff = (diffDis ? 1 : 0);

        simpleSlides(kUL*upLInput - kDiff*diffInput, kUR*upRInput + kDiff*diffInput);
        return new double[]{kUL*upLInput - kDiff*diffInput, kUR*upRInput + kDiff*diffInput}; // returns for telemetry, would be better as a getter but im lazy
    }

    public void moveSlide(int ticks) {
        final int maxPos = 5070;
        int sum = lSTargetPos + ticks;
        if (sum <= maxPos) lSTargetPos = sum;
    }

    public int getlSTargetPos(){
        return lSTargetPos;
    }

    public int[] getLSPos(){return new int[] {linearSlideLeft.getCurrentPosition(), linearSlideRight.getCurrentPosition()};}

    public void simpleSlides(double power1, double power2) {
        /*if (linearSlideAtLimit(linearSlideLeft.getCurrentPosition(), power1)) linearSlideLeft.setPower(0);
        else linearSlideLeft.setPower(power1);
        linearSlideLeft.setPower(power1);

        if (linearSlideAtLimit(linearSlideRight.getCurrentPosition(), power2)) linearSlideRight.setPower(0);
        else linearSlideRight.setPower(power2);*/
        linearSlideLeft.setPower(power1);
        linearSlideRight.setPower(power2);
    }

    private boolean linearSlideAtLimit(int pos, double pow) {
        return (pos > 5070 && pow > 0); //|| (pos < 0 && pow < 0);
    }

    public void setLSPIDCoeff(double p, double i, double d, LSType type) { // scales PID coefficients by parameter
        if (type == LSType.UP_LEFT) {
            upLKP = p;
            upLKI = i;
            upLKD = d;
            upLLSPID = new PIDController(p, i ,d);
        }
        else if (type == LSType.UP_RIGHT) {
            upRKP = p;
            upRKI = i;
            upRKD = d;
            upRLSPID = new PIDController(p, i ,d);
        }
        else {
            dKP = p;
            dKI = i;
            dKD = d;
            diffLSPID = new PIDController(p, i ,d);
        }
    }

    public double[] getLSPIDCoeff(LSType type) {
        if (type == LSType.UP_LEFT) {
            return new double[] {upLKP, upLKI, upLKD};
        }
        else if (type == LSType.UP_RIGHT) {
            return new double[] {upRKP, upRKI, upRKD};
        }
        else {
            return new double[] {dKP, dKI, dKD};
        }
    }

    public void disableLS(LSType type, boolean status) {
        if(type == LSType.UP_LEFT) {
            upLDis = status;
        }
        else if(type == LSType.UP_RIGHT) {
            upRDis = status;
        }
        else {
            diffDis = status;
        }
    }

    /*
                - MISC -
               ~ TELEOP ~
     */


    public void moveCarousel(double power) { // in is positive
        carousel.setPower(power);
    }

    public void moveInputWheel(double power) {
        input.setPower(power);
    }


    /*
                ~ AUTONOMOUS ~
     */


    public double[] getSensorBoolean() { // should return in order left-most to right-most
        return null;
    }


    public double getBarcodeDistance(DistanceUnit unit) {
        return barcode.getDistance(unit);
    }


    /*
                - UTILITIES -
     */


    public Position normalize(Position reference, Position transform, double angle) { // converts a transform from the perspective of angle
        if (!reference.unit.equals(transform.unit)) { // convert to same units
            reference = reference.toUnit(transform.unit);
        }
        Position diff = new Position(reference.unit, transform.x-reference.x, transform.y-reference.y, transform.x-reference.z, 0);
        diff = rotateTransform(diff, 90 - angle); // rotates it so that the robot is looking north (y is forward, x is sideways)
        return diff;
    }

    public Position rotateTransform(Position transform, double angle) { // rotates a 2d vector by angle
        double x = transform.x * Math.cos(angle) - transform.y * Math.sin(angle);
        double y = transform.x * Math.sin(angle) + transform.y * Math.cos(angle);
        return new Position(transform.unit, x, y, 0.0, 0); // TODO check this
    }

    public Position addPositions(Position pos1, Position pos2) {
        return new Position(DistanceUnit.INCH, pos1.x + pos2.x, pos1.y + pos2.y, pos1.z + pos2.z, 0);
    }

    public Position subtractPositions(Position pos1, Position pos2) {
        return new Position(DistanceUnit.INCH, pos1.x - pos2.x, pos1.y - pos2.y, pos1.z - pos2.z, 0);
    }
}
