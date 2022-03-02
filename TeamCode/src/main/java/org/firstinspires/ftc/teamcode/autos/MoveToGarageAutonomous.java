package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.TelemetryFactory;
import org.firstinspires.ftc.teamcode.movement.StateFactory;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Autonomous(name="Move to garage")
public class MoveToGarageAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        AbState move = new EncoderForwardState("Move to garage" , hardwareHandler, 48, 1);
        PlaceholderState placeholder = new PlaceholderState();
        move.putNextState("next", placeholder);
        AbState currState = move;
        hardwareHandler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();
            double[] motorPos = hardwareHandler.getMotorPositions();
            telemetry.addData("lf pos: ", motorPos[0]);
            telemetry.addData("rf pos: ", motorPos[1]);
            telemetry.addData("lr pos: ", motorPos[2]);
            telemetry.addData("rr pos: ", motorPos[3]);
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            for (TelemetryObj obj : TelemetryFactory.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.update();
        }
    }
}
