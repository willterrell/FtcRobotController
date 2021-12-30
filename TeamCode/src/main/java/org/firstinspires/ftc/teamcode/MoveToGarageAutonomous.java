package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="Move to garage")
public class MoveToGarageAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap);
        EncoderMove move = new EncoderMove("Move to garage" , hardwareHandler, new Position(DistanceUnit.METER, 0, 4, 0, 0), 0, 0.5);
        PlaceholderState placeholder = new PlaceholderState();
        move.putNextState("next", placeholder);
        AbState currState = move;
        waitForStart();
        hardwareHandler.moveSlide(7200);
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
