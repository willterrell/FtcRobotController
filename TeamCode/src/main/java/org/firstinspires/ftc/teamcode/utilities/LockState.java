package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.HashMap;

public class LockState extends AbState {
    // runs blocker and runner until blocker finishes, returns runner's next state
    private AbState blocker;
    private AbState runner;
    private boolean finished;
    public LockState(String name, AbState blocker, AbState runner) {
        super(name);
    }

    @Override
    public void init() {
        blocker.putNextState("next", new PlaceholderState());
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (finished) {
            return runner.getNextState("next");
        }
        else return this;
    }

    @Override
    public void run() {
        blocker.run();
        runner.run();
        if (blocker.next() instanceof PlaceholderState) finished = true;
    }
}
