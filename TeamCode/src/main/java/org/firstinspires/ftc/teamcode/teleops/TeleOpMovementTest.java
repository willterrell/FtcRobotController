package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

@TeleOp(name="TeleOpMovementTest")
public class TeleOpMovementTest extends OpMode {
    private HardwareHandler hardwareHandler;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
    }

    @Override
    public void loop() {
        double speed = (gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x); // magnitude squared
        speed = Math.max(Math.min(1, speed), -1);
        hardwareHandler.moveWithPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);
    }
}
