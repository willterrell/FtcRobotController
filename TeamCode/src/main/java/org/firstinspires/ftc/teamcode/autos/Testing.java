package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderStrafe;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;
import org.firstinspires.ftc.teamcode.utilities.CloseClawState;
import org.firstinspires.ftc.teamcode.utilities.MoveSlidesState;
import org.firstinspires.ftc.teamcode.utilities.OpenClawState;

import java.util.Arrays;

@Config
@Autonomous(name="testAutonomous")
public class Testing extends LinearOpMode {
    public static double STRAFE_TO_COLOR = -19;
    public static double STRAFE_TO_MID_DISTANCE = -45;
    public static double STRAFE_BACK_DISTANCE = 2;
    public static double TO_JUNCTION_DISTANCE = 6.5;
    public static double STRAFE_TO_TILE_DISTANCE = 33;
    public static double TO_GARAGE_1_DISTANCE = -24;
    public static double TO_GARAGE_2_DISTANCE = 0;
    public static double TO_GARAGE_3_DISTANCE = 24;
    public static double SPEED = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
        CloseClawState closeClaw = new CloseClawState("closeClaw", hardwareHandler);
        EncoderStrafe strafeToColor = new EncoderStrafe("strafeToColor", hardwareHandler, STRAFE_TO_COLOR, SPEED);
        EncoderStrafe strafeToMid = new EncoderStrafe("strafeToMid", hardwareHandler, STRAFE_TO_MID_DISTANCE, SPEED);
        EncoderStrafe strafeBack = new EncoderStrafe("strafeBack", hardwareHandler, STRAFE_BACK_DISTANCE, SPEED);
        MoveSlidesState slideUp = new MoveSlidesState("slideUp", hardwareHandler, SlidePosition.LARGE_JUNCTION);
        EncoderForwardState toJunction = new EncoderForwardState("toJunction", hardwareHandler, TO_JUNCTION_DISTANCE, SPEED);
        OpenClawState openClaw = new OpenClawState("openClaw", hardwareHandler);
        EncoderForwardState outOfJunction = new EncoderForwardState("outOfJunction", hardwareHandler, -TO_JUNCTION_DISTANCE, SPEED);
        MoveSlidesState slideDown = new MoveSlidesState("slideDown", hardwareHandler, SlidePosition.ONE_CONE);
        EncoderStrafe strafeToMiddleOfTile = new EncoderStrafe("strafeToMiddleOfTile", hardwareHandler, STRAFE_TO_TILE_DISTANCE, SPEED);


        closeClaw.putNextState("next", strafeToColor);
        strafeToColor.putNextState("next", new PlaceholderState());
        AbState currState = closeClaw;

        waitForStart();
        while (!(currState instanceof PlaceholderState)) {
            currState.run();
            currState = currState.next();
        }

        double[] color = hardwareHandler.getColorSensorReading();

        strafeToMid.putNextState("next", slideUp);
        slideUp.putNextState("next", toJunction);
        toJunction.putNextState("next", openClaw);
        openClaw.putNextState("next", outOfJunction);
        outOfJunction.putNextState("next", slideDown);
        slideDown.putNextState("next", strafeToMiddleOfTile);

        EncoderForwardState garage;
        String name = "None";
        // green
        if ((color[0] > 1200 && color[0] < 2000) && (color[1] > 1200 && color[1] < 2000)) {
            garage = new EncoderForwardState("Garage 2", hardwareHandler, TO_GARAGE_2_DISTANCE, SPEED);
            name = "green";
        }
        // pink
        else if ((color[0] > 3500 && color[0] < 4500) && (color[1] > 3500 && color[1] < 4000) && (color[2] > 4300)) {
            garage = new EncoderForwardState("Garage 3", hardwareHandler, TO_GARAGE_3_DISTANCE, SPEED);
        }
        // pink
        else if ((color[0] > 200 && color[0] < 1000) && (color[1] > 500 && color[1] < 800) && (color[2] < 1000)) {
            garage = new EncoderForwardState("Garage 1", hardwareHandler, TO_GARAGE_1_DISTANCE, SPEED);
            name = "pink";
        }
        else {
            garage = new EncoderForwardState("Error", hardwareHandler, TO_GARAGE_2_DISTANCE, SPEED);
            name = "purple";
        }

        telemetry.addData("color", Arrays.toString(color));
        telemetry.addData("type", name);

        strafeToMiddleOfTile.putNextState("next", garage);
        garage.putNextState("next", new PlaceholderState());

        currState = strafeToMid;

        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();

            telemetry.addData("AbState name: ", currState.getName());
            telemetry.addData("Is busy: ", hardwareHandler.driveTrainIsBusy());
            telemetry.update();

            if (currState instanceof PlaceholderState) break;
        }
    }
}
