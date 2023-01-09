package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareHandler;

@TeleOp(name="FloatSlideTest")
public class FloatSlideTest extends OpMode {
    private HardwareHandler hardwareHandler;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        hardwareHandler.setSlidePower(gamepad1.left_stick_y * 0.1);
    }
}
