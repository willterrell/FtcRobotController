package org.firstinspires.ftc.teamcode.utilities.CenterOnPole;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.HashMap;

@Config
public class CenterOnPoleState extends AbState {
    private HardwareHandler hardwareHandler;
    private SampleMecanumDrive drive;
    public static double TOLERANCE = 0.2, VELOCITY = 0.1, ACCELERATION = 0.5, EMPTY_DISTANCE = 7.4, SEARCH_DISTANCE = 5, DISTANCE_FROM_POLE = 5;
    public static TrajectoryVelocityConstraint SLOW_VEL = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL * VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    public static TrajectoryAccelerationConstraint SLOW_ACC = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * ACCELERATION);
    private AbState currState;
    private boolean record, noCones;
    public CenterOnPoleState(String name, HardwareHandler hardwareHandler, boolean record, boolean noCones) {
        super(name, "next", "error");
        this.hardwareHandler = hardwareHandler;
        this.drive = hardwareHandler.getDrive();
        this.record = record;
        this.noCones = noCones;
    }

    @Override
    public void init() {
        //FindPole findPole = new FindPole("findPole", hardwareHandler, true);
        CenterY centerY = new CenterY("centerY", hardwareHandler, SEARCH_DISTANCE, noCones, false);
        CenterY centerY2 = new CenterY("centerY2", hardwareHandler, SEARCH_DISTANCE * 2, noCones, true);
        CenterX centerX = new CenterX("centerX", hardwareHandler, noCones);
        //findPole.putNextState("next", centerX);
        //findPole.putNextState("error", getNextState("next")); // if we cant find the pole, just do what we were going to do anyway
        centerY.putNextState("next", centerX);
        centerY2.putNextState("next", centerX);
        centerY.putNextState("error", centerY2);
        centerY2.putNextState("error", new PlaceholderState("error"));
        centerX.putNextState("next", new PlaceholderState("next"));
        currState = centerY;
        currState.init();
        setSubStates(currState);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (currState instanceof PlaceholderState) {
            if (currState.getName().equals("error")) return getNextState("error");
            if (record) {
                drive.setPoseEstimateAndTrajEnd(getPoseEstimateGivenAtAPole());
            }
            return getNextState("next");
        }
        return this;
    }

    @Override
    public void run() {
        currState.run();
        AbState nextState = currState.next();
        if (!nextState.equals(currState)) {
            setSubStates(nextState);
        }
        currState = nextState;
    }

    private Pose2d getPoseEstimateGivenAtAPole() {
        double robotAngle = hardwareHandler.getIMUZAngle();
        double angleFromPole = (robotAngle - 180);
        double xFromPole = Math.cos(Math.toRadians(angleFromPole)) * DISTANCE_FROM_POLE; // robot coordinates with respect to the pole we're at
        double yFromPole = Math.sin(Math.toRadians(angleFromPole)) * DISTANCE_FROM_POLE;
        Pose2d currPose = drive.getPoseEstimate();
        double currPoleX = currPose.getX() - xFromPole; // get coordinate
        double currPoleY = currPose.getY() - yFromPole;
        double closestPoleX = Math.round(currPoleX / 24) * 24;
        double closestPoleY = Math.round(currPoleY / 24) * 24;
        return new Pose2d(closestPoleX + xFromPole, closestPoleY + yFromPole, Math.toRadians(robotAngle));
    }
}
