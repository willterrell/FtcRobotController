package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class CarouselMoveState extends AbState {
    private ElapsedTime timer;
    private double startTime;
    private final double deltaTime;
    private final HardwareHandler hardwareHandler;
    private final double power;
    public CarouselMoveState(String name, HardwareHandler hardwareHandler, double power, double time) {
        super(name, "next");
        deltaTime = time;
        this.hardwareHandler = hardwareHandler;
        this.power = power;
    }

    @Override
    public void init() {
        startTime = timer.milliseconds();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (timer.milliseconds() > startTime + deltaTime) return nextStateMap.get("next");
        else return this;
    }

    @Override
    public void run() {
        hardwareHandler.moveCarousel(power);
    }
}