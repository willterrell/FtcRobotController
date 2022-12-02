package org.firstinspires.ftc.teamcode.movement.encoder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class EncoderWaitState extends AbState {
    private List<DcMotor> motors;
    private ElapsedTime timer;
    public EncoderWaitState(String name, List<DcMotor> motors) {
        super(name, "next");
        motors = this.motors;
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return this;
            }
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        return nextStateMap.get("next");
    }

    @Override
    public void run() {

    }
}
