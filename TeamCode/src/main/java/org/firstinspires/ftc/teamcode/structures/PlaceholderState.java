package org.firstinspires.ftc.teamcode.structures;

import org.firstinspires.ftc.teamcode.AbState;

import java.util.HashMap;

public class PlaceholderState extends AbState { // this state does nothing
    public PlaceholderState() {
        super("Placeholder");
    }

    public PlaceholderState(String name) {
        super(name);
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return this;
    }

    @Override
    public void run() {

    }
}
