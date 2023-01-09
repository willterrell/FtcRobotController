package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

@Config
@TeleOp(name="Claw Test")
public class ClawTest extends OpMode {
    public static double LEFT = 0, RIGHT = 0;
    private HardwareHandler hardwareHandler;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
    }

    @Override
    public void loop() {
        hardwareHandler.moveClaw(LEFT, RIGHT);
    }
}
