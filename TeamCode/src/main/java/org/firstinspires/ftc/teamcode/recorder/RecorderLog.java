package org.firstinspires.ftc.teamcode.recorder;

import android.content.ClipData;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.K;

import java.security.KeyException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * RecorderLog provides an object for storing RecorderItem objects and finding their specified actions
 *
 * @author wrtc9
 */

public class RecorderLog {
    private ArrayList<RecorderItem> items; // could make more sense to have items be a hashmap
    private final transient Map<String, Action> actions; // this won't necessarily be init
    private final Set<String> actionAlphabet; // this could just be set to actions.keySet
    private final double secondsPerTick;

    public RecorderLog(Set<String> actionAlphabet, double ticksPerSecond) {
        this.actionAlphabet = actionAlphabet;
        actions = new HashMap<String, Action>();
        secondsPerTick = 1000/ticksPerSecond;
    }

    public void putItem(RecorderItem item) throws ActionNotIncluded {
        double timeMS = item.getTimeMS();
        for (String name : item.getNames()) { // ensure every action in the item is part of the known actions
            if (!actionAlphabet.contains(name)) throw new ActionNotIncluded(name);
        }
        int index = (int) Math.round(timeMS/secondsPerTick);
        // if so, add it
        items.add(index, item);
    }

    @NonNull
    public RecorderItem getItem(double time) throws ItemNotFound {
        int index = (int) Math.round(time/secondsPerTick);
        RecorderItem item = items.get(index);
        if (item != null) return item; // null checking
        else throw new ItemNotFound(Integer.toString(index));
    }

    public ArrayList<Runnable> getActionsWithParameters(double timeMS) throws ActionNotIncluded {
        if (actions.keySet().containsAll(actionAlphabet)) { // given actions should have all desired actions

            // finds the item for a given time
            RecorderItem item;
            try {
                item = getItem(timeMS);
            }
            catch (ItemNotFound e) {
                item = new RecorderItem(timeMS);
                // its alright if they don't have the specified time, just do nothing for that time
            }

            // finds all of the actions associated with the item and returns them as a list of runnables
            // runnable = function that takes no parameters and returns no values
            ArrayList<Runnable> executionList = new ArrayList<>();
            for (String parameterName : item.getNames()) {
                Object[] paramters = item.getParameter(parameterName);
                Action action = actions.get(parameterName);
                Runnable runnable = () -> action.execute(paramters);
                executionList.add(runnable);
            }
            return executionList;
        } // if there are not actions included, record which actions are not
        else {
            Set<String> notIncluded = new HashSet<String>(actionAlphabet);
            notIncluded.removeAll(actions.keySet());
            throw new ActionNotIncluded(notIncluded.toString());
        }
    }

    public void addAction(Action action) throws ActionNotIncluded {
        String name = action.getName();
        if (actionAlphabet.contains(name)) { // only include an action if it is in the desired actions list
            actions.put(name, action);
        }
        else throw new ActionNotIncluded(name); // you can just avoid adding it
    }

    public double findLastTime() { // if we change the map to a list, we can instead just return length * spt
        return (items.size()-1) * secondsPerTick;
    }
}
