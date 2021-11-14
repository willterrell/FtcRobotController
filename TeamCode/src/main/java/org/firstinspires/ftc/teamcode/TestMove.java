package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.HashMap;

@Autonomous(name="Test Move", group="tests")
public class TestMove extends LinearOpMode {
    private HardwareHandler hardwareHandler;
    private TankMoveState tankMoveState;
    private AbState currState;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareHandler = new HardwareHandler(hardwareMap);
        tankMoveState = new TankMoveState("test move", hardwareHandler, new Position(DistanceUnit.METER, 1.0, 0.0, 0.0, 0), 45);
        tankMoveState.putNextState("next", new PlaceholderState());
        currState = tankMoveState;

        waitForStart();

        while (!currState.getClass().equals(PlaceholderState.class) && opModeIsActive()) {
            currState.run();
            currState = currState.next();
            telemetry.addData("Position:", hardwareHandler.getIMUPosition());
            telemetry.addData("Rotation:", hardwareHandler.getIMUZAngle());
        }
    }
}
