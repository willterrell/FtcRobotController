package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderStrafe;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

//@Autonomous(name="Move test 2")
public class MoveToGarageAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        AbState move = new EncoderStrafe("Move to garage" , hardwareHandler, 48, 1);
        PlaceholderState placeholder = new PlaceholderState();
        move.putNextState("next", placeholder);
        AbState currState = move;
        hardwareHandler.setDriveTrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();
            double[] motorPos = hardwareHandler.getEncoderPositions();
            telemetry.addData("lf pos: ", motorPos[0]);
            telemetry.addData("rf pos: ", motorPos[1]);
            telemetry.addData("lr pos: ", motorPos[2]);
            telemetry.addData("rr pos: ", motorPos[3]);
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.update();
        }
    }
}
