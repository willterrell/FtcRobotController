package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Consumer;

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
        ArrayList<String> points = new ArrayList<>();
        for (PathSegment seg : trajectory.getPath().getSegments()) {
            points.add(seg.end().toString());
        }
        addTele(new TelemetryObj("traj", points.toString()));
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
