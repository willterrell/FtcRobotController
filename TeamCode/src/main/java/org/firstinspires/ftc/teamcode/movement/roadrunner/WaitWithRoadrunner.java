package org.firstinspires.ftc.teamcode.movement.roadrunner;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.HashMap;

public class WaitWithRoadrunner extends AbState {
    SampleMecanumDrive drive;

    public WaitWithRoadrunner(String name, SampleMecanumDrive drive) {
        super(name, "next");
        this.drive = drive;
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (!drive.isBusy()) {
            return this.getNextState("next");
        }
        return this;
    }

    @Override
    public void run() {

    }
}
