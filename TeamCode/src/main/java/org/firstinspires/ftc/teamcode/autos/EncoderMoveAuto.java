package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Autonomous(name="EncoderMove")
public class EncoderMoveAuto extends LinearOpMode {
    private Position addPos(Position ...terms) {
        Position ans = new Position(terms[0].unit, 0, 0, 0, 0);
        for (Position term : terms) {
            ans = new Position(ans.unit, ans.x+term.x, ans.y+term.y, ans.z+term.z, 0);
        }
        return ans;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        final double speed = 0.3;
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        AbState currState;
        Position currPos = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        EncoderMove move1 = new EncoderMove("move1", hardwareHandler, addPos(currPos, new Position(DistanceUnit.METER, 1, 1, 0, 0)), 45, speed, PosType.RELATIVE);
        EncoderMove move2 = new EncoderMove("move2", hardwareHandler, addPos(currPos, new Position(DistanceUnit.METER, 0, 2, 0, 0)), 270, speed, PosType.RELATIVE);
        move1.putNextState("next", move2);
        move2.putNextState("next", new PlaceholderState());
        currState = move1;

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
