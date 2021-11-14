package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Locale;
import java.util.Set;

public abstract class AbState { // this is basically a decorator pattern
    protected String name;
    protected AbState currentState = this;
    protected HashMap<String, AbState> nextStateMap;

    public AbState(String name, String ...nextStateName){
        this.name = name;
        for (String stateName : nextStateName) {
            nextStateMap.put(stateName, null); // this should be set with putNextState
        }
    }

    public String getName() {
        return name;
    }

    public AbState getCurrentState(){
        return currentState;
    }

    public abstract void init(); // expected to be run in next

    public AbState next() {
        for (String nextName : nextStateMap.keySet()) {
            assert (nextStateMap.get(nextName) != null): String.format(Locale.ENGLISH, "In %s, next state, %s, is not defined", name, nextName);
        }
        return nextImpl();
    }

    public abstract AbState nextImpl(); // returns next state to be run and also end behavior

    public abstract void run();

    public void putNextState(String name, AbState state){ // probably a better way to set next state than constructor
        if (nextStateMap.containsKey(name)) {
            nextStateMap.put(name, state);
        }
    }

    public Set<String> getNextStateKeys() {
        return nextStateMap.keySet();
    }
}