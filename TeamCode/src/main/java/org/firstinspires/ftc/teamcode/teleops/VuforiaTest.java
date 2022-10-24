package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.vuforia.VuforiaHandler;

//@TeleOp(name="Vuforia Test")
public class VuforiaTest extends OpMode {
    private HardwareHandler hardwareHandler;
    private VuforiaHandler vuforiaHandler;
    @Override
    public void init() {
        vuforiaHandler = new VuforiaHandler();
    }

    @Override
    public void loop() {
        vuforiaHandler.update();
    }
}
