package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

@Config
public class ConeStackToHighJunction extends AbState {
    private SampleMecanumDrive drive;
    public static double MIDDLE_X = 24, MIDDLE_Y = 12, MIDDLE_DEG = -75;
    private Pose2d MIDDLE;
    public static double HIGH_X = 24, HIGH_Y = 8.5, HIGH_DEG = -90; // START_X = 38.75, START_Y = 64.5, START_DEG = 180;
    private Pose2d HIGH_JUNCTION;
    private AbState currState;
    public ConeStackToHighJunction(String name, SampleMecanumDrive drive) {
        super(name, "next");
        this.drive = drive;
    }

    @Override
    public void init() {
        MIDDLE = new Pose2d(MIDDLE_X, MIDDLE_Y, Math.toRadians(MIDDLE_DEG));
        HIGH_JUNCTION = new Pose2d(HIGH_X, HIGH_Y, Math.toRadians(HIGH_DEG));
        Trajectory traj1 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .lineToLinearHeading(MIDDLE)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(HIGH_JUNCTION)
                .build();
        MoveWithRoadrunner move1 = new MoveWithRoadrunner("startToMiddle", traj1, drive);
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("startToMiddle", traj2, drive);
        move1.putNextState("next", move2);
        move2.putNextState("next", getNextState("next"));
        currState = move1;
        currState.init();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return currState;
    }

    @Override
    public void run() {

    }
}
