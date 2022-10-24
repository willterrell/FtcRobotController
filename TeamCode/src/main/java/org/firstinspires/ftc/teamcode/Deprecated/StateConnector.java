package org.firstinspires.ftc.teamcode.Deprecated;

import org.firstinspires.ftc.teamcode.AbState;
import org.json.*;

import java.io.File;
import java.util.HashMap;
import java.util.Locale;

@Deprecated
public class StateConnector {
    private HashMap<String, AbState> states;

    public StateConnector(){} // to use this, one should first put all of their states into the hashmap, then connect them via connectViaJson

    public void connectViaJson(File file) throws JSONException { // uses a Json file to set nextStateMap in AbStates
        String rawText = file.toString();
        String filename = file.getName();
        JSONObject jsonObj = new JSONObject(rawText);

        for (String name : states.keySet()) { // iterates through all states to be connected
            AbState state = states.get(name);
            assert state != null; // I don't think this will ever happen

            assert (jsonObj.has(name)): String.format(Locale.ENGLISH, "JSON in %s does not contain state, %s", filename, name);
            JSONObject connections = jsonObj.getJSONObject(name);

            for (String conName : state.getNextStateKeys()) { // goes through each required connection and sets that connection
                assert (connections.has(conName)): String.format(Locale.ENGLISH, "JSON in %s in %s does not contain connection %s", filename, name, conName);
                String stateName = connections.getString(conName);

                assert(states.containsKey(stateName)): String.format(Locale.ENGLISH, "JSON in %s; %s is not in states", filename, stateName);
                AbState nextState = states.get(stateName);

                state.putNextState(conName, nextState);
            }
        }
    }

    public void putState(String name, AbState state) {
        states.put(name, state);
    }
}
