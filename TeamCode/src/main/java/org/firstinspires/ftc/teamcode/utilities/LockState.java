package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.ArrayList;
import java.util.HashMap;

public class LockState extends AbState {
    // WIP, running from array does not work?
    private ArrayList<AbState> states;
    private boolean finished;
    public LockState(String name, ArrayList<AbState> states) {
        super(name, "next");
        this.states = states;
    }

    @Override
    public void init() {
        for (AbState state : states) {
            state.putNextState("next", new PlaceholderState());
            state.init();
        }
        setSubStates(states);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        finished = true;
        for (AbState state : states) {
            finished &= state instanceof PlaceholderState;
        }
        if (finished) return getNextState("next");
        else return this;
    }

    @Override
    public void run() {
        for (int i = 0; i < states.size(); i++) {
            AbState state = states.get(i);
            state.run();
            states.set(i, state.next());
        }
    }
}
