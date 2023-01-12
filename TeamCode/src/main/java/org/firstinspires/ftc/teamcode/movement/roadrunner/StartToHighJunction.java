package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;
import org.firstinspires.ftc.teamcode.utilities.LockState;
import org.firstinspires.ftc.teamcode.utilities.MoveSlidesState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

@Config
public class StartToHighJunction extends AbState {
    public static double MIDDLE_X = 39.5, MIDDLE_Y = 12, MIDDLE_DEG = 180;
    private Pose2d MIDDLE;
    public static double HIGH_X = 24, HIGH_Y = 8.5, HIGH_DEG = -90;
    private Pose2d HIGH_JUNCTION;
    private HardwareHandler hardwareHandler;
    private SampleMecanumDrive drive;
    private AbState currState;
    public StartToHighJunction(String name, HardwareHandler hardwareHandler) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.drive = hardwareHandler.getDrive();
    }

    @Override
    public void init() {
        MIDDLE = new Pose2d(MIDDLE_X, MIDDLE_Y, Math.toRadians(MIDDLE_DEG));
        Trajectory traj1 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .lineToLinearHeading(MIDDLE)
                .build();
        HIGH_JUNCTION = new Pose2d(HIGH_X, HIGH_Y, Math.toRadians(HIGH_DEG));
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(HIGH_JUNCTION,
                        SampleMecanumDrive.getVelocityConstraint(0.5 * DriveConstants.MAX_VEL, 0.5 * DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(0.5 * DriveConstants.MAX_ACCEL))
                .build();
        MoveWithRoadrunner move1 = new MoveWithRoadrunner("startToMiddle", traj1, drive);
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("middleToHigh", traj2, drive);
        MoveSlidesState slides = new MoveSlidesState("slideToHigh", hardwareHandler, SlidePosition.LARGE_JUNCTION);
        LockState lock = new LockState("middleToHigh + slideToHigh", new ArrayList<>(Arrays.asList(move2, slides)));
        move1.putNextState("next", lock);
        lock.putNextState("next", getNextState("next"));
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
