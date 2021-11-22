package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class PlaceholderState extends AbState { // this state does nothing
    public PlaceholderState() {
        super("Placeholder");
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
