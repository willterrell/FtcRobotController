package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

public class MoveWithRoadrunner extends AbState {
    private Trajectory trajectory;
    private SampleMecanumDrive drive;

    public MoveWithRoadrunner(String name, Trajectory trajectory, SampleMecanumDrive drive) {
        super(name, "next");
        this.trajectory = trajectory;
        this.drive = drive;
    }


    @Override
    public void init() {
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (drive.isBusy()) return this;
        return getNextState("next");
    }

    @Override
    public void run() {
    }
}
