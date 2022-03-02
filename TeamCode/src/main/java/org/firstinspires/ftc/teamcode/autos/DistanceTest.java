package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.TelemetryFactory;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Autonomous(name="movement test")
public class DistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        EncoderMove forward1 = new EncoderMove("move1", hardwareHandler, new Position(DistanceUnit.INCH, 0, 12*12, 0, 0), 180, 1, PosType.RELATIVE);
        EncoderMove forward2 = new EncoderMove("move2", hardwareHandler, new Position(DistanceUnit.INCH, 0, 12*12, 0, 0), 180, 1, PosType.RELATIVE);
        EncoderMove forward3 = new EncoderMove("move3", hardwareHandler, new Position(DistanceUnit.INCH, 0, 12*12, 0, 0), 180, 1, PosType.RELATIVE);
        forward1.putNextState("next", forward2);
        forward2.putNextState("next", forward3);
        forward3.putNextState("next", new PlaceholderState());
        AbState currState = forward1;
        currState.init();
        hardwareHandler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();

            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.addData("Is busy: ", hardwareHandler.isBusy());
            telemetry.update();
        }
    }
}
