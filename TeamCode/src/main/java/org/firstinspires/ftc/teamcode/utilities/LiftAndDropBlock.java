package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.DropOffPosition;
import org.firstinspires.ftc.teamcode.structures.ExpelOrInput;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.Side;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

public class LiftAndDropBlock extends AbState { // TODO implement this; ls height will be especially important
    private HardwareHandler hardwareHandler;
    private EncoderMove moveToHub;
    private RaiseLS raise, lower;
    private ExpelBlock expel;
    private Side side;
    private int dropOffIn;
    private AbState currState;
    private TelemetryObj dropOffPosTele;
    public LiftAndDropBlock(String name, HardwareHandler hardwareHandler, DropOffPosition pos, Side side) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.side = side;
        if (pos == DropOffPosition.BOTTOM_TIER) dropOffIn = 1350;
        else if (pos == DropOffPosition.MIDDLE_TIER) dropOffIn = 3150; // update these vals
        else dropOffIn = 4950;
        dropOffPosTele = new TelemetryObj("Tier", pos.toString());
    }

    @Override
    public void init() {
        double angle = 90;
        Position hubPos = new Position(); // TODO UPDATE HUBPOS
        if (side == Side.RED) {
            angle = -90;
            hubPos = new Position();
        }
        //moveToHub = new EncoderMove("moveToHub", hardwareHandler, hubPos, angle, 0.2, PosType.RELATIVE); // NEED POSITION
        raise = new RaiseLS("raiseLS", hardwareHandler, dropOffIn);
        expel = new ExpelBlock("expel", hardwareHandler, ExpelOrInput.EXPEL, 2000);
        lower = new RaiseLS("lowerLS", hardwareHandler, 0);
        //moveToHub.putNextState("next", raise);
        raise.putNextState("next", expel);
        expel.putNextState("next", lower);
        lower.putNextState("next", new PlaceholderState());
        currState = raise;
        currState.init();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (currState instanceof PlaceholderState) return nextStateMap.get("next");
        return this;
    }

    @Override
    public void run() {
        currState.run();
        currState = currState.next();
        setSubStates(currState);
    }
}
