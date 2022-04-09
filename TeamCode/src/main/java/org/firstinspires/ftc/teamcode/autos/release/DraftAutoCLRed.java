package org.firstinspires.ftc.teamcode.autos.release;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderStrafe;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.BarcodeState;
import org.firstinspires.ftc.teamcode.utilities.CarouselMoveState;
import org.firstinspires.ftc.teamcode.utilities.LiftAndDropBlock;
import org.firstinspires.ftc.teamcode.utilities.RaiseLS;

@Autonomous(name="Barcode + Carousel Red")
public class DraftAutoCLRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(DistanceUnit.INCH, 61.125, -49.5, 0, 0), 90);
        // changes : moveToCarousel, moveToHub, moveToAlign, speed

        EncoderMove moveBit = new EncoderMove("moveBit", hardwareHandler, new Position(DistanceUnit.INCH, 0, 2, 0, 0), 0, 1, PosType.RELATIVE);
        RaiseLS raise = new RaiseLS("raise", hardwareHandler, 500);
        EncoderStrafe moveToCarousel = new EncoderStrafe("moveToCarousel", hardwareHandler, 18, 1);
        CarouselMoveState carousel = new CarouselMoveState("moveCarousel", hardwareHandler, 1, 3000);
        EncoderStrafe moveToMiddle1 = new EncoderStrafe("moveToMiddle", hardwareHandler, -44, 1);

        EncoderMove moveToHub = new EncoderMove("moveToHub", hardwareHandler, new Position(DistanceUnit.INCH, 0,17, 0, 0), 0, 1, PosType.RELATIVE);
        BarcodeState doBarcode = new BarcodeState("doBarcode", hardwareHandler, Side.BLUE);
        EncoderMove rotate = new EncoderMove("rotate", hardwareHandler, new Position(), -90, 1, PosType.RELATIVE);

        EncoderStrafe moveToGarage1 = new EncoderStrafe("moveToGarage1", hardwareHandler, -24, 1);
        EncoderMove moveToGarage2 = new EncoderMove("moveToGarage2", hardwareHandler, new Position(DistanceUnit.INCH, 0, 48, 0, 0), 0, 1, PosType.RELATIVE);

        moveToHub.putNextState("next", doBarcode);
        doBarcode.putNextState("next", rotate);
        rotate.putNextState("next", moveToGarage1);

        moveToGarage1.putNextState("next",moveToGarage2);
        moveToGarage2.putNextState("next", new PlaceholderState());

        AbState currState = moveBit;
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
