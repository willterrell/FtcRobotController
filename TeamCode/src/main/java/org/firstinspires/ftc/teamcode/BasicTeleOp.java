package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Basic TeleOp")
public class BasicTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = (Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2))/2;
        hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, 0, speed);
    }
}
