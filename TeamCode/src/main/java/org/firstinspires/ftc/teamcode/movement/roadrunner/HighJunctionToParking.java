package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.ParkingPosition;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;
import org.firstinspires.ftc.teamcode.utilities.LockState;
import org.firstinspires.ftc.teamcode.utilities.MoveSlidesState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

@Config
public class HighJunctionToParking extends AbState {
    private SampleMecanumDrive drive;
    private HardwareHandler hardwareHandler;
    public static double MIDDLE_X = 24, MIDDLE_Y = 14, MIDDLE_DEG = -75;
    public static double PARKING_Y = 14, ONE_X = 60, TWO_X = 36, THREE_X = 12, PARKING_DEG = 0; // 1 = leftmost, 2 = middle, 3 = rightmost
    private final double PARKING_X;
    public static double RIGHT_MIDDLE_X = -24;
    private AbState currState;
    private Side side;
    public HighJunctionToParking(String name, HardwareHandler hardwareHandler, ParkingPosition parkingPosition, Side side) {
        super(name, "next");
        this.drive = hardwareHandler.getDrive();
        this.hardwareHandler = hardwareHandler;
        switch (parkingPosition) {
            case ONE:
                PARKING_X = ONE_X;
                break;
            case TWO:
                PARKING_X = TWO_X;
                break;
            default:
                PARKING_X = THREE_X;
        }
        this.side = side;
    }

    @Override
    public void init() {
        Pose2d MIDDLE, PARKING;
        if (side == Side.LEFT) {
            MIDDLE = new Pose2d(MIDDLE_X, MIDDLE_Y, Math.toRadians(MIDDLE_DEG));
            PARKING = new Pose2d(PARKING_X, PARKING_Y, Math.toRadians(PARKING_DEG));
        }
        else {
            MIDDLE = new Pose2d(RIGHT_MIDDLE_X, MIDDLE_Y, Math.toRadians(MIDDLE_DEG + 180));
            PARKING = new Pose2d(-PARKING_X, PARKING_Y, Math.toRadians(PARKING_DEG + 180));
        }
        Trajectory traj1 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .lineToLinearHeading(MIDDLE)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(PARKING)
                .build();
        MoveWithRoadrunner move1 = new MoveWithRoadrunner("startToMiddle", traj1, drive);
        MoveSlidesState slide = new MoveSlidesState("slideToDown", hardwareHandler, SlidePosition.ONE_CONE);
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("startToMiddle", traj2, drive);
        move1.putNextState("next", slide);
        slide.putNextState("next", move2);
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
