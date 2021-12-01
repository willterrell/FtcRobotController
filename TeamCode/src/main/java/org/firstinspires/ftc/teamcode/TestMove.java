package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="Test Move", group="tests")
public class TestMove extends LinearOpMode {
    private HardwareHandler hardwareHandler;
    private MoveWithIMUState moveWithIMUState;
    private AbState currState;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareHandler = new HardwareHandler(hardwareMap);
        moveWithIMUState = new MoveWithIMUState("test move", hardwareHandler, new Position(DistanceUnit.METER, 1.0, 0.0, 0.0, 0), 45);
        moveWithIMUState.putNextState("next", new PlaceholderState());
        currState = moveWithIMUState;


        waitForStart();

        while (!currState.getClass().equals(PlaceholderState.class) && opModeIsActive()) {
            currState.run();
            currState = currState.next();
            telemetry.addData("Position:", hardwareHandler.getIMUPosition());
            telemetry.addData("Rotation:", hardwareHandler.getIMUZAngle());
        }
    }
}
