package org.firstinspires.ftc.teamcode.recorder;

import androidx.annotation.NonNull;

public class ActionNotIncluded extends Exception {
    private String name;
    public ActionNotIncluded(String actionName) {
        name = actionName;
    }
    @NonNull
    @Override
    public String toString() {
        return name + " has not been included as an action";
    }
}
