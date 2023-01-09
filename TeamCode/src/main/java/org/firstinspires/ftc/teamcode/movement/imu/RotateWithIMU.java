package org.firstinspires.ftc.teamcode.movement.imu;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PIDFController;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;
import java.util.Locale;

@Config
public class RotateWithIMU extends AbState {
    private HardwareHandler hardwareHandler;
    private PIDFController pid;
    private double currAngle;
    private ElapsedTime timer;
    private double prevTime;
    private boolean rising = false, prevRise = false;
    private TelemetryObj targetAngTele, pidInputTele, errorTele, angTele;
    private TelemetryObj pidSpecificsTele;
    public static double p = 0.03, i = 0, d = -0.05, c = 0.04, iD = 0, ANGLE = 180;
    public static double ANGLE_ACCURACY = 1, SPEED_LIMIT = 0.1, TIME_WAIT = 100;
    private double targetAngle = ANGLE; // 0.02 0.001 0.006 0.075
    private double initAng;

    //TODO Look into feedforward pid

    public RotateWithIMU(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        targetAngTele = new TelemetryObj("Target Angle: ", targetAngle);
        pidInputTele = new TelemetryObj("PID Input: ");
        pidSpecificsTele = new TelemetryObj("PID Specifics: ");
        errorTele = new TelemetryObj("Error: ");
        angTele = new TelemetryObj("Angle: ");
    }

    public RotateWithIMU(String name, HardwareHandler hardwareHandler, double angle, double speed) {
        this(name, hardwareHandler);
        this.hardwareHandler = hardwareHandler;
        targetAngle = angle % 360;
        if (targetAngle < 0) targetAngle += 360;
    }

    @Override
    public void init() { //0.001 ; 0.0125, 0.00035, 0.03 is good
        pid = new PIDFController(p, i, d, iD, c, 0, 0.75); // 0.004, 0.004, 0.00005, 0.1
        currAngle = hardwareHandler.getIMUZAngle();
        addTele(targetAngTele);
        addTele(pidInputTele);
        addTele(pidSpecificsTele);
        addTele(errorTele);
        addTele(angTele);
        timer = new ElapsedTime();
        initAng = hardwareHandler.getIMUZAngle();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        double error = targetAngle - currAngle;
        if (Math.abs(targetAngle - currAngle) > 180) error -= 360 * Math.signum(error);
        boolean bool = Math.abs(error) <= ANGLE_ACCURACY;
        rising = bool && !prevRise;
        prevRise = bool;
        if (rising) { // within 5 degrees of target
            timer.reset();
        }
        if (Math.abs(hardwareHandler.getPowers()[0]) < SPEED_LIMIT && timer.milliseconds() > TIME_WAIT && bool) {
            hardwareHandler.setDriveTrainPowers(0, 0, 0, 0);
            return nextStateMap.get("next");
        }
        return this;
    }

    @Override
    public void run() {
        currAngle = hardwareHandler.getIMUZAngle() - initAng;
        if (currAngle < 0) currAngle += 360;
        double error = targetAngle - currAngle;
        if (Math.abs(targetAngle - currAngle) > 180) error -= 360 * Math.signum(error);
        double input = pid.getInput(error);
        pidInputTele.setContent(input);
        errorTele.setContent(error);
        angTele.setContent(currAngle);
        pidSpecificsTele.setContent(String.format(Locale.ENGLISH, "p:%f, i:%f, d:%f", pid.getP(), pid.getI(), pid.getD()));
        //if (Math.abs(input) > 1) input = Math.signum(input); // clips input
        hardwareHandler.moveWithPower(0, -Math.signum(input), 0, input); //Math.max(Math.min(input, 1), -1) * speed
    }
}
