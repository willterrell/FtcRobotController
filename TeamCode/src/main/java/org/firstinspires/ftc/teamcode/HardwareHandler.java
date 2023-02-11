package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.movement.imu.SimpsonIntegrator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.PIDFController;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import kotlin.Unit;

@Config
public class HardwareHandler {
    private final HardwareMap hardwareMap;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private BNO055IMU imu;
    private SimpsonIntegrator integrator;
    private final int msPollInterval = 100;
    private final Servo leftClawServo, rightClawServo;
    private ColorSensor colorSensor;

    private DcMotorEx linearSlide;
    private DistanceSensor clawSensor;

    private DistanceSensor leftPoleSensor;
    private DistanceSensor rightPoleSensor;

    public double recordedLeftPole, recordedRightPole;

    public static int ONE_CONE_POS = 0, TWO_CONE_POS = 110, THREE_CONE_POS = 220, FOUR_CONE_POS = 330, FIVE_CONE_POS = 440,
                      SMALL_JUNCTION_POS = 1350, MEDIUM_JUNCTION_POS = 2195, LARGE_JUNCTION_POS = 3060;
    public static double SLIDE_TICK_PER_POWER = 2000;
    public static double KLF = 1, KLR = 0.95, KRF = 0.9, KRR = 0.855; //KLF = 1, KLR = 0.944, KRF = 0.934825, KRR = 0.8824748;
    public static double SLIDE_SPEED = 0.5;
    public static int SLIDE_TOLERANCE = 10;
    public static double CLAW_OPEN = 0.15;
    public static double lsP = 0.00325, lsI = 0, lsD = 0, lsG = 0.001;


    private int[] slidePresetPositions = new int[] {ONE_CONE_POS, TWO_CONE_POS, THREE_CONE_POS, FOUR_CONE_POS, FIVE_CONE_POS, SMALL_JUNCTION_POS, MEDIUM_JUNCTION_POS, LARGE_JUNCTION_POS};
    private ArrayList<SlidePosition> slidePosNames = new ArrayList<SlidePosition>(Arrays.asList(SlidePosition.values()));
    private int slidePositionIndex = 0;
    private int targetSlidePos = 0;
    private PIDFController slideController;

    private SampleMecanumDrive drive;

    /*
    TODO
     change wheel diameter and robot length
     */
    private boolean prevHalt = false, prevPower = false;

    private DcMotor.RunMode currRunMode;

    private Position currPos;
    private double initAngle;

    private Telemetry telemetry;

    private PIDFController rotationController;

    // TODO INITIAL ANGLE

    public HardwareHandler(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, new Position(), telemetry);
    }

    public HardwareHandler(HardwareMap hardwareMap, Position currPos, Telemetry telemetry) {
        this(hardwareMap, currPos, 0, telemetry);
    }

    public HardwareHandler(HardwareMap hardwareMap, Position currPos, double initAngle, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        // Motor initiations here
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");

        leftClawServo = hardwareMap.servo.get("leftClawServo");
        rightClawServo = hardwareMap.servo.get("rightClawServo");

        linearSlide = (DcMotorEx) hardwareMap.dcMotor.get("linearSlide");

        clawSensor = hardwareMap.get(DistanceSensor.class, "clawSensor");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        leftPoleSensor = hardwareMap.get(DistanceSensor.class, "leftPoleSensor");
        rightPoleSensor = hardwareMap.get(DistanceSensor.class, "rightPoleSensor");

        leftClawServo.scaleRange(-1, 1);
        rightClawServo.scaleRange(-1, 1);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideController = new PIDFController(lsP, lsI, lsD, 0, lsG, 0);

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

        this.currPos = currPos;
        this.initAngle = initAngle;
        this.telemetry = telemetry;
    }


    /*
                - DRIVETRAIN -
                  ~ TELEOP ~
     */

    public void moveWithPower(double d, double r, double s, double speed) { // d : linear movement, r : rotational movement, s : speed (0-1); r is signed with CCW as positive
        //assert (speed <= 1 && speed >= 0): "Speed must be between 0 and 1";
        if (currRunMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) setDriveTrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed = Math.abs(speed);
        double total = Math.abs(d) + Math.abs(r) + Math.abs(s);
        if (d == 0 && r == 0 && s == 0) {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        else {
            leftFront.setPower((-d + r + s) / total * speed * KLF);
            leftRear.setPower((-d + r - s) / total * speed * KLR); // 0.88 gotten from testing
            rightFront.setPower((-d - r - s) / total * speed * KRF);
            rightRear.setPower((-d - r + s) / total * speed * KRR);

            /*
            d + r + s
            d + r - s
            d - r - s
            d - r + s
             */
        }
    }

    public void avoidJunctions(boolean lfBumper, boolean lrBumper, boolean rfBumper, boolean rrBumper, double f, double s) {
        if (!lfBumper && !lrBumper && !rfBumper && !rrBumper) return;
        boolean goingForward = f > s;

        if (lfBumper) {

        }
    }

    public double[] getVelocities() {
        return new double[]{leftFront.getVelocity(), leftRear.getVelocity(), rightFront.getVelocity(), rightRear.getVelocity()};
    }

    public double[] getPowers() {
        return new double[]{leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower()};
    }

    public ArrayList<DcMotor> getMotors() {
//        ArrayList<DcMotor> arr = new ArrayList<>();
//        arr.add(leftFront);
//        arr.add(leftRear);
//        arr.add(rightFront);
//        arr.add(rightRear);
        return new ArrayList<>(Arrays.asList(leftFront, leftRear, rightFront, rightRear));
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
    public Position getVirtualPosition() {
        return currPos;
    }

    public double[] getEncoderPositions() {return new double[]{leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};}

    public void addToVirtualPosition(Position pos, double angle) {
        pos = findRelativeDifference(new Position(DistanceUnit.INCH, 0, 0, 0, 0), pos, angle); // accounts for angle when adding position
        double x = currPos.x + pos.x * Math.cos(Math.toDegrees(angle)), y = currPos.y + pos.y * Math.sin(Math.toDegrees(angle));
        x = (Math.abs(x) > 70.125) ? 70.125 * Math.signum(x) : x; // clips position to the field
        y = (Math.abs(y) > 70.125) ? 70.125 * Math.signum(y) : y;
        currPos = new Position(DistanceUnit.INCH, x, y, currPos.z, 0);
    }

    public void setVirtualPosition(Position pos) {
        currPos = pos;
    }

    public void setDriveTrainPowers(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    public boolean driveTrainIsBusy() {
        return leftFront.isBusy() ||
                rightFront.isBusy() ||
                leftRear.isBusy() ||
                rightRear.isBusy();
    }

    public void setDriveTrainMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftRear.setMode(mode);
        rightRear.setMode(mode);
        currRunMode = mode;
    }

    public void setDriveTrainEncoderTargets(int tickLF, int tickLR, int tickRF, int tickRR) {
        setDriveTrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition((int)((tickLF + leftFront.getCurrentPosition()))); // 3.125 is for gearing
        rightFront.setTargetPosition((int)((tickRF + rightFront.getCurrentPosition())));
        leftRear.setTargetPosition((int)((tickLR + leftRear.getCurrentPosition())));
        rightRear.setTargetPosition((int)((tickRR + rightRear.getCurrentPosition())));

//        leftFront.setTargetPosition(-tickLF); // 3.125 is for gearing
//        rightFront.setTargetPosition(-tickRF);
//        leftRear.setTargetPosition(-tickLR);
//        rightRear.setTargetPosition(-tickRR);

        setDriveTrainMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setEncoderTargetsForward(int ticks, double power) {
        setDriveTrainEncoderTargets(ticks, ticks, ticks, ticks);
        setDriveTrainPowers(power, power, power, power);
    }

    public void setEncoderTargetsStrafe(int ticks, double power) {
        setDriveTrainEncoderTargets(ticks, (int)(-ticks * 0.9895), -ticks, (int)(ticks * 0.9895));
        setDriveTrainPowers(power, power * 0.9825, power, power * 0.9825);
    }

    @Deprecated
    public void setEncoderTargetsRotate(int ticks, double power) { // CCW is +
        setDriveTrainEncoderTargets(ticks, -ticks, ticks, -ticks);
        setDriveTrainPowers(power, power, power, power);
    }

    public void goForwardWithEncoders(double distance, double power) { // only needs to be run once
        int ticks = (int) (distance * 3000 / 66.5); // TODO Maybe fit this to y=mx+b instead of y=mx for stopping error']

        setEncoderTargetsForward(ticks, power);
    }

    public void strafeWithEncoders(double distance, double power) {
        int ticks = (int) (distance * 5911 / 121.5);
        setEncoderTargetsStrafe(ticks, power);
    }

    /*
    @Deprecated
    public void rotateWithEncoders(double degrees) {
        double inPerDegree = Math.PI * Math.sqrt(length*length + width*width) / 360;
        double inPerTick = ticksPerRotation * Math.PI * wheelDiameter; // ticks * mpt / mpd = degrees
        int ticks = (int) (degrees / inPerTick * inPerDegree);
        setEncoderTargetsRotate(ticks);
    }
     */



    public void stopDrivetrain() {
        setDriveTrainPowers(0, 0, 0, 0);
        setDriveTrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getTargetDistance() {
        return leftFront.getTargetPosition();
    }

    public SampleMecanumDrive getDrive() {
        return drive;
    }

    public void setDrive(SampleMecanumDrive drive) {this.drive = drive;}


    /*
                - IMU -
     */


    public void initIMU(Velocity currVelocity) { // should be called on the start of the opmode
        imu.startAccelerationIntegration(currPos, currVelocity, msPollInterval); // example had it with 1000ms?
    }

    public double getIMUZAngle() {
        double rawAngle = initAngle + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        rawAngle %= 360;
        if (rawAngle < 0) rawAngle += 360;
        return rawAngle;
    }

    public Position getIMUPosition() { // gives current position as a double list formatted [x, y, r]
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

    public void resetIMUZAngle() {
        initAngle = getIMUZAngle();
    }

    public double getNearest90() {
        double angle = getIMUZAngle();
        // find first 90 degree angle that is greater than current angle
        telemetry.addData("angle", angle);
        double closestAngle = 0;
        double closestDiff = 360;
        for (int i = 0; i < 4; i++) {
            double testAngle = 90 * i;
            // consider angle that are congruent to test angle as well, for when testAngle = 0/360
            double diff = Math.min(Math.abs(testAngle - angle), Math.min(Math.abs(testAngle + 360 - angle), Math.abs(testAngle - 360 - angle)));
            if (diff < closestDiff) {
                closestDiff = diff;
                closestAngle = testAngle; // find the first angle that is greater than our angle
            }
        }
        return closestAngle;
    }

    public double getNearest90AboveOrBelow(boolean below) {
        double angle = getIMUZAngle();
        double closest90 = getNearest90();
        if (angle < closest90 && below) closest90 -= 90;
        if (angle > closest90 && !below) closest90 += 90;
        if (Math.abs(closest90 - angle) < 1) closest90 += (below) ? -90:90;
        return closest90;
    }


    /*
                - LINEAR SLIDES -
     */

    public void moveSlidesToPreset(SlidePosition pos) {
        switch (pos) {
            case ONE_CONE:
                targetSlidePos = ONE_CONE_POS;
                break;
            case TWO_CONE:
                targetSlidePos = TWO_CONE_POS;
                break;
            case THREE_CONE:
                targetSlidePos = THREE_CONE_POS;
                break;
            case FOUR_CONE:
                targetSlidePos = FOUR_CONE_POS;
                break;
            case FIVE_CONE:
                targetSlidePos = FIVE_CONE_POS;
                break;
            case SMALL_JUNCTION:
                targetSlidePos = SMALL_JUNCTION_POS;
                break;
            case MEDIUM_JUNCTION:
                targetSlidePos = MEDIUM_JUNCTION_POS;
                break;
            case LARGE_JUNCTION:
                targetSlidePos = LARGE_JUNCTION_POS;
                break;
            default:
                targetSlidePos = 0;
                break;
        }
//        linearSlide.setTargetPosition(targetSlidePos);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(SLIDE_SPEED); // what power?
    }

    public int getNearestSlidePosIndex(int position) {
        int closestIndex = 0;
        int closestDistance = 5000;
        for (int i = slidePresetPositions.length - 1; i >= 0; i--) {
            int distance = Math.abs(position - slidePresetPositions[i]);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    public int getNearestSlidePosAboveOrBelow(int position, boolean below) {
        int nearestIndex = getNearestSlidePosIndex(position);
        int nearestPosition = slidePresetPositions[nearestIndex];
        if (position < nearestPosition && below) nearestIndex -= 1;
        if (position > nearestPosition && !below) nearestIndex += 1;
        nearestIndex = Math.max(Math.min(nearestIndex, slidePresetPositions.length - 1), 0);
        nearestPosition = slidePresetPositions[nearestIndex];
        if (Math.abs(nearestPosition - position) < SLIDE_TOLERANCE) nearestIndex += (below) ? -1 : 1;
        nearestIndex = Math.max(Math.min(nearestIndex, slidePresetPositions.length - 1), 0);
        return slidePresetPositions[nearestIndex];
    }

    public boolean slideBusy() {
        return linearSlide.isBusy();
    }

    /*public void addSlideSetpoint(int ticks, boolean lowerLimit) {
        final int maxPos = 2500;
        int sum = lsSetpoint + ticks;
        if (sum <= maxPos && (sum >= 0 || !lowerLimit)) lsSetpoint = sum;
        if (prevLowerLimit && !lowerLimit) {
            initSlidePos = linearSlide.getCurrentPosition();
            lsSetpoint = 0;
        }
        prevLowerLimit = lowerLimit;
    }*/

    public void moveSlide(double power, boolean useLimiter, Telemetry telemetry) {
        boolean atLimit = areLinearSlideAtLimit(linearSlide.getCurrentPosition(), -power - lsG)
                          && useLimiter;
        telemetry.addData("atLimit", atLimit);
        telemetry.addData("slide power", linearSlide.getPower());
        telemetry.addData("slide pos", linearSlide.getCurrentPosition());
        //double floatingPower = -0.01; // tune floatingPower until slides hold their position at setPower(floatingPower)
        if ((power == 0 || atLimit) && !prevHalt) { // rising edge of not moving
            haltSlides(); // could
            telemetry.addData("halting", true);
        }
        if (power != 0 && !atLimit) { // moving
            if (linearSlide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            power = Math.max(Math.min(power, 1), -1);
            linearSlide.setPower(-power - lsG);
        }
        if (power == 0 || atLimit) {
            updateSlides();
        }
        prevHalt = power == 0 || atLimit;
    }

    public void moveSlidesHybrid(double power, boolean dpadUp, boolean dpadDown, boolean useLimiter, boolean top, boolean medium, boolean low, boolean bottom) {
        boolean atLimit = areLinearSlideAtLimit(linearSlide.getCurrentPosition(), power + lsG)
                && useLimiter;
        telemetry.addData("prevPower", prevPower);
        telemetry.addData("ls target", linearSlide.getTargetPosition());
        telemetry.addData("ls mode", linearSlide.getMode());
        if(power != 0 || prevPower) {
            moveSlide(power, useLimiter, telemetry);
            telemetry.addData("driver controlled", true);
            prevPower = true;
        }
        if (dpadUp || dpadDown) { // dpad supposed to be the rising edge
            int newPos = getNearestSlidePosAboveOrBelow(linearSlide.getCurrentPosition(), dpadDown);
            targetSlidePos = newPos;
            telemetry.addData("preset", true);
            prevPower = false;
        }
        if (top) {
            targetSlidePos = slidePresetPositions[slidePresetPositions.length - 1];
            prevPower = false;
        }
        else if (medium) {
            targetSlidePos = slidePresetPositions[slidePresetPositions.length - 2];
            prevPower = false;
        }
        else if (low) {
            targetSlidePos = slidePresetPositions[slidePresetPositions.length - 3];
            prevPower = false;
        }
        else if (bottom) {
            targetSlidePos = slidePresetPositions[0];
            prevPower = false;
        }
        if (power == 0) {
            updateSlides();
        }
    }

    public void haltSlides() {
        // hold position of slide (only run on rising edge)
//        if (linearSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        linearSlide.setTargetPosition(linearSlide.getCurrentPosition()); // use PID to maintain current position
//        linearSlide.setPower(0.01); // what power to use?
        targetSlidePos = linearSlide.getCurrentPosition();
    }

    private boolean areLinearSlideAtLimit(int pos, double pow) {
        return (pos > 3060 && pow > 0) || (pos < 0 && pow < 0);
    }

    public void setSlidePower(double power) {
        linearSlide.setPower(power);
    }

    public boolean slidesAtTarget() {
        return Math.abs(linearSlide.getTargetPosition() - linearSlide.getCurrentPosition()) < SLIDE_TOLERANCE;
    }

    public void shiftSlidePosition(boolean dpadUp, boolean dpadDown, boolean fullUp, boolean fullDown) {
        slidePositionIndex += ((dpadUp) ? 1 : 0) + ((dpadDown) ? -1 : 0);
        slidePositionIndex %= slidePosNames.size();
        if (slidePositionIndex < 0) slidePositionIndex += slidePosNames.size();
        if (fullUp) {
            slidePositionIndex = slidePosNames.size() - 1;
        }
        if (fullDown) {
            slidePositionIndex = 0;
        }
        telemetry.addData("Slide Position", slidePosNames.get(slidePositionIndex));
        moveSlidesToPreset(slidePosNames.get(slidePositionIndex));
    }

    public void updateSlides() {
        double pos = linearSlide.getCurrentPosition();
        if (Math.abs(pos) < SLIDE_TOLERANCE && Math.abs(targetSlidePos) < SLIDE_TOLERANCE) linearSlide.setPower(0);
        else {
            double error = targetSlidePos - pos;
            double input = slideController.getInput(error);
            linearSlide.setPower(input);
        }
    }

    public boolean slidesAtPosition() {
        return Math.abs(linearSlide.getCurrentPosition() - targetSlidePos) < SLIDE_TOLERANCE;
    }

    public DcMotor getSlideMotor() {
        return linearSlide;
    }

    public int getTargetSlidePos() {return linearSlide.getCurrentPosition();}

    /*public void setLSPIDCoeff(double p, double i, double d, double id, double c, LSType type) { // scales PID coefficients by parameter
        if (type == LSType.UP_LEFT) {
            upLKP = p;
            upLKI = i;
            upLKD = d;
            upLKID = id;
            upLKC = c;
            upLLSPID = new PIDController(p, i ,d, id, c, 100);
        }
        else if (type == LSType.UP_RIGHT) {
            upRKP = p;
            upRKI = i;
            upRKD = d;
            upRKID = id;
            upRKC = c;
            upRLSPID = new PIDController(p, i ,d, id, c, 100);
        }
        else {
            dKP = p;
            dKI = i;
            dKD = d;
            dKID = id;
            dKC = c;
            diffLSPID = new PIDController(p, i ,d, id, c, 100);
        }
    }*/

    /*
                - MISC -
               ~ TELEOP ~
     */

    public double getClawSensorDistance(DistanceUnit unit) {
        return clawSensor.getDistance(unit);
    }

    public void moveClaw(double leftPosition, double rightPosition) {
        leftClawServo.setPosition(leftPosition);
        rightClawServo.setPosition(rightPosition);
    }

    public void openClaw() {
        moveClaw(0.14, 0.88);
    }

    public void closeClaw() {
        moveClaw(0.05, .951);
    }

    public double[] getColorSensorReading() {
        double[] reading = new double[3];
        reading[0] = colorSensor.red();
        reading[1] = colorSensor.blue();
        reading[2] = colorSensor.green();
        return reading;
    }

    public double getLeftPoleSensor(DistanceUnit unit) {
        recordedLeftPole = leftPoleSensor.getDistance(unit);
        return recordedLeftPole;
    }

    public double getRightPoleSensor(DistanceUnit unit) {
        recordedRightPole = rightPoleSensor.getDistance(unit);
        return recordedRightPole;
    }

    public double getRecordedLeftPoleSensor() {
        return recordedLeftPole;
    }

    public double getRecordedRightPoleSensor() {
        return  recordedRightPole;
    }

    /*
                ~ AUTONOMOUS ~
     */


    /*
                - UTILITIES -
     */


    public Position findRelativeDifference(Position robotPos, Position targetPos, double robotAngle) { // converts a transform from the perspective of angle
        if (!robotPos.unit.equals(targetPos.unit)) { // convert to same units
            robotPos = robotPos.toUnit(targetPos.unit);
        }
        Position diff = new Position(robotPos.unit, targetPos.x-robotPos.x, targetPos.y-robotPos.y, targetPos.x-robotPos.z, 0);
        diff = rotatePositionAroundOrigin(diff, 90 - robotAngle); // rotates difference so that y is forward and x is side to side
        return diff;
    }

    public Position rotatePositionAroundOrigin(Position transform, double angle) { // rotates a 2d vector by angle
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
