package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.BarcodeState;
import org.firstinspires.ftc.teamcode.utilities.CarouselMoveState;
import org.firstinspires.ftc.teamcode.utilities.LiftAndDropBlock;

@Autonomous
public class DraftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(DistanceUnit.INCH, 61.125, -49.5, 0, 0));
        EncoderMove moveToCarousel = new EncoderMove("moveToCarousel", hardwareHandler, new Position(DistanceUnit.INCH, 0, -12, 0, 0), 0, 0.5, PosType.RELATIVE);
        CarouselMoveState moveCarousel = new CarouselMoveState("moveCarousel", hardwareHandler, 1, 1000);
        EncoderMove moveToMiddle1 = new EncoderMove("moveToMiddle", hardwareHandler, new Position(DistanceUnit.INCH, 61.125, -11.688, 0, 0), 90, 0.5, PosType.ABSOLUTE);
        EncoderMove moveToAlign = new EncoderMove("moveToAlign", hardwareHandler, new Position(DistanceUnit.INCH, 0, -9, 0, 0), 90, 0.3, PosType.RELATIVE);
        //hardwareHandler.setEncoderPosition(new Position(DistanceUnit.INCH, 70.125, -11.688, 0, 0));
        EncoderMove moveToHub = new EncoderMove("moveToHub", hardwareHandler, new Position(DistanceUnit.INCH, 23.375, -11.688, 0, 0), 90, 0.5, PosType.ABSOLUTE);
        BarcodeState barcodeState = new BarcodeState("barcode", hardwareHandler);
        EncoderMove moveToMiddle2 = new EncoderMove("moveToMiddle", hardwareHandler, new Position(DistanceUnit.INCH, 61.125, -11.688, 0, 0), 0, 0.5, PosType.ABSOLUTE);
        EncoderMove moveToGarage = new EncoderMove("moveToHub", hardwareHandler, new Position(DistanceUnit.INCH, 0, 50, 0, 0), 0,1, PosType.RELATIVE);
        moveToCarousel.putNextState("next", moveCarousel);
        moveCarousel.putNextState("next", moveToMiddle1);
        moveToMiddle1.putNextState("next", moveToAlign);
        moveToAlign.putNextState("next", moveToHub);
        moveToHub.putNextState("next", barcodeState);
        barcodeState.putNextState("next", moveToMiddle2);
        moveToMiddle2.putNextState("next", moveToGarage);
        AbState currState = moveToCarousel;
        currState.init();

        waitForStart();

        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();

            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.update();
        }
    }
}
