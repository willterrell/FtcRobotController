package org.firstinspires.ftc.teamcode.autos.release;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderColorScan;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderStrafe;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.BarcodeState;

import java.util.Base64;

@Autonomous(name="Color Auto")
public class DraftColorAuto extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(DistanceUnit.INCH, 61.125, -49.5, 0, 0), 90);
        // changes : moveToCarousel, moveToHub, moveToAlign, speed
        EncoderColorScan coneColor = new EncoderColorScan("detectCone", hardwareHandler);

        EncoderMove moveRed = new EncoderMove("moveRed", hardwareHandler, new Position(DistanceUnit.INCH, 0, 2, 0, 0), 0, 0.5, PosType.RELATIVE);
        EncoderMove moveBlue = new EncoderMove("moveBlue", hardwareHandler, new Position(DistanceUnit.INCH, 0, -2, 0, 0), 0, 1, PosType.RELATIVE);
        EncoderMove moveGreen = new EncoderMove("moveGreen", hardwareHandler, new Position(DistanceUnit.INCH, 0, 5, 0, 0), 0, 1, PosType.RELATIVE);

        coneColor.putNextState("red", moveRed);


        AbState currState = moveRed;
        currState.init();

        waitForStart();

        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();

            for (TelemetryObj obj : currState.getTelemetries()) { // null check telemetry list
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.update();
        }
    }
    //EncoderMove moveToAlign = new EncoderMove("moveToAlign", hardwareHandler, new Position(DistanceUnit.INCH, 5, 0, 0, 0), -90, 0.5, PosType.RELATIVE);
    //hardwareHandler.setEncoderPosition(new Position(DistanceUnit.INCH, 70.125, -11.688, 0, 0));
    //EncoderMove moveToMiddle2 = new EncoderMove("moveToMiddle", hardwareHandler, new Position(DistanceUnit.INCH, 0, -10, 0, 0), 90, 1, PosType.RELATIVE);
    //EncoderMove moveToGarage = new EncoderMove("moveToHub", hardwareHandler, new Position(DistanceUnit.INCH, 0, 50, 0, 0), 0,1, PosType.RELATIVE);
    //moveToAlign.putNextState("next", moveToHub);
}


