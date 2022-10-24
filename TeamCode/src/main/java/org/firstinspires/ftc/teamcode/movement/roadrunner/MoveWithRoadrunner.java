package org.firstinspires.ftc.teamcode.movement.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

public class MoveWithRoadrunner extends AbState {
    Trajectory trajectory;
    SampleMecanumDrive drive;
    WaitWithRoadrunner wait;

    public MoveWithRoadrunner(String name, Trajectory trajectory, SampleMecanumDrive drive) {
        super(name, "next");
        this.trajectory = trajectory;
        this.drive = drive;
    }


    @Override
    public void init() {
        AbState next = this.getNextState("next");
        wait = new WaitWithRoadrunner(name + " Wait", drive);
        wait.putNextState("next", next);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return wait;
    }

    @Override
    public void run() {
        drive.followTrajectoryAsync(trajectory);
    }
}
