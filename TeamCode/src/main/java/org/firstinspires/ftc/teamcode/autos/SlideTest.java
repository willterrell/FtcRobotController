package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.DropOffPosition;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.BarcodeState;
import org.firstinspires.ftc.teamcode.utilities.LiftAndDropBlock;
import org.firstinspires.ftc.teamcode.utilities.RaiseLS;

//@Autonomous(name="Slide Test")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        //LiftAndDropBlock lift = new LiftAndDropBlock("lift", hardwareHandler, DropOffPosition.TOP_TIER, Side.RED);
        //RaiseLS lift = new RaiseLS("raise", hardwareHandler, 1000);
        BarcodeState barcode = new BarcodeState("barcode", hardwareHandler, Side.RED);
        barcode.putNextState("next", new PlaceholderState());
        AbState currState = barcode;
        currState.init();
        hardwareHandler.setDriveTrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();

            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("AbState name: ", currState.getName());
            telemetry.addData("Is busy: ", hardwareHandler.driveTrainIsBusy());
            telemetry.update();
        }
    }
}
