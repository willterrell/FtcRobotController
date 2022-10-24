package org.firstinspires.ftc.teamcode.recorder;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashMap;

public class RecorderState extends AbState {
    private final HardwareHandler hardwareHandler;
    private final Gamepad gamepad1, gamepad2;
    public RecorderState(String name, HardwareHandler hardwareHandler, Gamepad gamepad1, Gamepad gamepad2, ) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /* Idea:
        * alphabet = actions.keySet
        * when recording, actions is used
        * need an object that translates gamepad inputs -> parameters 
     */

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return null;
    }

    @Override
    public void run() {

    }
}
