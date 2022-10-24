package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.MoveWithIMUState;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;

@Deprecated
@Autonomous(name="Deprecated test move", group="deprecated")
public class TestMove extends LinearOpMode {
    private HardwareHandler hardwareHandler;
    private MoveWithIMUState moveWithIMUState;
    private AbState currState;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        moveWithIMUState = new MoveWithIMUState("test move", hardwareHandler, new Position(DistanceUnit.METER, 1.0, 0.0, 0.0, 0), 45, 0.3, PosType.RELATIVE);
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
