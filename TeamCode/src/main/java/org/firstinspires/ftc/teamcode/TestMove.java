package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name="Test Move", group="tests")
public class TestMove extends OpMode {
    private HardwareHandler hardwareHandler;
    private TankMoveState tankMoveState;
    private AbState currState;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
        tankMoveState = new TankMoveState("test move", new PlaceholderState(), hardwareHandler, new Position(DistanceUnit.METER, 1.0, 0.0, 0.0, 0), 45);
        currState = tankMoveState;
    }

    @Override
    public void loop() {
        if (!currState.getClass().equals(PlaceholderState.class)) {
            currState.run();
        }
    }
}
