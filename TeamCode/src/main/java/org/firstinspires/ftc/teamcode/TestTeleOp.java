package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.Speed;

@TeleOp(name="TestTeleOp", group="TeleOp")
public class TestTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;
    private final double Speed = 0.3;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
    }

    @Override
    public void loop() {
        hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, Speed);
    }
}
