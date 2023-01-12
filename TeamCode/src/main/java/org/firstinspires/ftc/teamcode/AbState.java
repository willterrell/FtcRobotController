package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;

import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.lang.reflect.Array;
import java.security.KeyException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Set;

public abstract class AbState { // this is basically a decorator pattern
    protected String name;
    private ArrayList<AbState> subStates;
    private final HashMap<String, AbState> nextStateMap;
    private final ArrayList<TelemetryObj> teleArr;
    public AbState(String name, String ...nextStateName){
        this.name = name;
        nextStateMap = new HashMap<String, AbState>();
        for (String stateName : nextStateName) {
            nextStateMap.put(stateName, null); // this should be set with putNextState
        }
        teleArr = new ArrayList<TelemetryObj>();
        subStates = new ArrayList<AbState>();
    }

    public String getName() {
        return name;
    }

    public ArrayList<TelemetryObj> getTelemetries() {
        ArrayList<TelemetryObj> telesIncludingSub = new ArrayList<>();
        //telesIncludingSub.add(new TelemetryObj("Number of Substates", subStates.length));
        telesIncludingSub.add(new TelemetryObj("Name", name));
        telesIncludingSub.add(new TelemetryObj("Type", this.getClass().getSimpleName()));
        telesIncludingSub.addAll(teleArr);
        for (AbState state : subStates) {
            telesIncludingSub.add(new TelemetryObj("",""));
            telesIncludingSub.addAll(state.getTelemetries());
        }
        return telesIncludingSub;
    }

    protected void addTele(TelemetryObj tele) {
        teleArr.add(tele);
    }

    protected void addAllTeles(ArrayList<TelemetryObj> teles) {
        teleArr.addAll(teles);
    }

    protected void setSubStates(ArrayList<AbState> abStates){
        subStates = abStates;
    }

    protected void setSubStates(AbState abState) {
        subStates.clear();
        subStates.add(abState);
    }

    public abstract void init(); // expected to be run in next

    public AbState next() {
        for (AbState possibleNext : nextStateMap.values()) {
            // check if null?
        }
        AbState nextState = next(nextStateMap);
        if (!nextState.equals(this)) {
            nextState.init();
        }
        return nextState;
    }

    public abstract AbState next(HashMap<String, AbState> nextStateMap); // returns next state to be run and also end behavior

    public abstract void run();

    public void putNextState(String name, AbState state){ // probably a better way to set next state than constructor
        if (nextStateMap.containsKey(name)) {
            nextStateMap.put(name, state);
        }
    }

    public Set<String> getNextStateKeys() {
        return nextStateMap.keySet();
    }

    public AbState getNextState(String name) {
        return nextStateMap.get(name);
    }
}