package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

@TeleOp(name="Claw Test")
public class ClawTest extends OpMode {
    private HardwareHandler hardwareHandler;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
    }

    @Override
    public void loop() {
        hardwareHandler.moveClawToMax();
    }
}
