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

//@Autonomous(name="Draft Auto C Red")
public class DraftAutoCRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(DistanceUnit.INCH, 61.125, -49.5, 0, 0), 90);
        EncoderMove moveToCarousel = new EncoderMove("moveToCarousel", hardwareHandler, new Position(DistanceUnit.INCH, 0, -23, 0, 0), 0, 0.5, PosType.RELATIVE);
        EncoderMove moveToGarage = new EncoderMove("moveToHub", hardwareHandler, new Position(DistanceUnit.INCH, 0, 100, 0, 0), 0,3, PosType.RELATIVE);
        moveToCarousel.putNextState("next", moveToGarage);

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
