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
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.CloseClawState;
import org.firstinspires.ftc.teamcode.utilities.MoveSlidesState;
import org.firstinspires.ftc.teamcode.utilities.OpenClawState;

@Config
@Autonomous(name="testAutonomous")
public class Testing extends LinearOpMode {
    public static double STRAFE_DISTANCE = -43;
    public static double STRAFE_TO_COLOR = -19;
    public static double TO_JUNCTION_DISTANCE = 6.5;
    public static double STRAFE_BACK_DISTANCE = 33;
    public static double TO_GARAGE_1_DISTANCE = -24;
    public static double TO_GARAGE_2_DISTANCE = 0;
    public static double TO_GARAGE_3_DISTANCE = 24;
    public static double SPEED = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
        EncoderStrafe strafeToColor = new EncoderStrafe("strafeToColor", hardwareHandler, STRAFE_TO_COLOR, SPEED);
        CloseClawState closeClaw = new CloseClawState("closeClaw", hardwareHandler);
        EncoderStrafe strafeToMid = new EncoderStrafe("strafeToMid", hardwareHandler, STRAFE_DISTANCE, SPEED);
        MoveSlidesState slideUp = new MoveSlidesState("slideUp", hardwareHandler, SlidePosition.LARGE_JUNCTION);
        EncoderForwardState toJunction = new EncoderForwardState("toJunction", hardwareHandler, TO_JUNCTION_DISTANCE, SPEED);
        OpenClawState openClaw = new OpenClawState("openClaw", hardwareHandler);
        EncoderForwardState outOfJunction = new EncoderForwardState("outOfJunction", hardwareHandler, -TO_JUNCTION_DISTANCE, SPEED);
        MoveSlidesState slideDown = new MoveSlidesState("slideDown", hardwareHandler, SlidePosition.ONE_CONE);
        EncoderStrafe strafeToMiddleOfTile = new EncoderStrafe("strafeToMiddleOfTile", hardwareHandler, STRAFE_BACK_DISTANCE, SPEED);
        EncoderForwardState garage = new EncoderForwardState("moveToGarage", hardwareHandler, TO_GARAGE_1_DISTANCE, SPEED);



        AbState currState = strafeToMid;

        waitForStart();

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
