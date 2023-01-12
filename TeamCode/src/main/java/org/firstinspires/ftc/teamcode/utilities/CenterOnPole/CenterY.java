package org.firstinspires.ftc.teamcode.utilities.CenterOnPole;

import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.ACCELERATION;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.EMPTY_DISTANCE;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_ACC;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_VEL;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.VELOCITY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.HashMap;

@Config
public class CenterY extends AbState {
    private HardwareHandler hardwareHandler;
    private SampleMecanumDrive drive;
    private double leftDistance, distance, prevDistance, prev2Distance;
    private TrajectoryVelocityConstraint SLOW_VEL = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL * VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private TrajectoryAccelerationConstraint SLOW_ACC = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * ACCELERATION);
    private AbState currState;
    private double searchDistance;
    private boolean noCones, backFirst;
    CenterY(String name, HardwareHandler hardwareHandler, double distance, boolean noCones, boolean backFirst) {
        super(name, "next", "error");
        this.hardwareHandler = hardwareHandler;
        this.drive = hardwareHandler.getDrive();
        searchDistance = distance;
        this.noCones = noCones;
        this.backFirst = backFirst;
    }

    @Override
    public void init() {
        leftDistance = hardwareHandler.getLeftPoleSensor(DistanceUnit.INCH);
        prevDistance = leftDistance;
        prev2Distance = prevDistance;
        searchDistance *= (backFirst) ? -1 : 1;
        Trajectory traj1 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .forward(searchDistance, SLOW_VEL, SLOW_ACC)
                .build();
        MoveWithRoadrunner move1 = new MoveWithRoadrunner("back", traj1, drive);
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(2 * searchDistance, SLOW_VEL, SLOW_ACC)
                .build();
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("forth", traj2, drive);
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(searchDistance, SLOW_VEL, SLOW_ACC)
                .build();
        MoveWithRoadrunner move3 = new MoveWithRoadrunner("backToStart", traj3, drive);
        move1.putNextState("next", move2);
        move2.putNextState("next", move3);
        move3.putNextState("next", new PlaceholderState());
        currState = move1;
        currState.init();
        setSubStates(currState);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (distance > prevDistance || noCones && distance < EMPTY_DISTANCE) {
            drive.breakFollowing();
            return getNextState("next");
        }
        if (currState instanceof PlaceholderState)
            return getNextState("error");
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
        leftDistance = hardwareHandler.getLeftPoleSensor(DistanceUnit.INCH);
        if (distance < EMPTY_DISTANCE) distance = (leftDistance + hardwareHandler.getRightPoleSensor(DistanceUnit.INCH));
        else distance = leftDistance;
        /*
        if (distance > prevDistance && prevDistance > prev2Distance && distance < EMPTY_DISTANCE) { // if we're increasing
            currState = currState.getNextState("next");
            setSubStates(currState);
        }
         */
        prevDistance = distance;
    }
}
