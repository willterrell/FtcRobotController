package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.StateFactory;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PosType;

@Autonomous(name="Move to garage")
public class MoveToGarageAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        AbState move = StateFactory.getMoveState("Move to garage" , hardwareHandler, new Position(DistanceUnit.METER, 0, 4, 0, 0), 0, 0.5, PosType.RELATIVE);
        PlaceholderState placeholder = new PlaceholderState();
        move.putNextState("next", placeholder);
        AbState currState = move;
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double prevTime = timer.milliseconds();
        while(opModeIsActive()) {
            double currTime = timer.milliseconds();
            currState.run();
            currState = currState.next();
            hardwareHandler.updateSlides();
            telemetry.addData("dt:", currTime - prevTime);
            prevTime = currTime;
        }
    }
}
