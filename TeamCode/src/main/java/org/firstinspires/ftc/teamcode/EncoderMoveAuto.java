package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap);
        AbState currState;
        Position currPos = new Position(DistanceUnit.METER, 0, 0, 0, 0);
        EncoderMove move1 = new EncoderMove("move1", hardwareHandler, addPos(currPos, new Position(DistanceUnit.METER, 1, 1, 0, 0)), 45, speed);
        EncoderMove move2 = new EncoderMove("move2", hardwareHandler, addPos(currPos, new Position(DistanceUnit.METER, 0, 2, 0, 0)), 270, speed);
        move1.putNextState("next", move2);
        move2.putNextState("next", new PlaceholderState());
        currState = move1;

        waitForStart();

        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
        }
    }
}
