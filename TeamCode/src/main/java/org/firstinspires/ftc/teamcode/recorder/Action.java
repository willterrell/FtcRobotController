package org.firstinspires.ftc.teamcode.recorder;

import java.io.Serializable;

public abstract class Action implements Serializable {
    private String name;
    public Action(String name) {
        this.name = name;
    }
    abstract public void execute(Object[] parameters);
    public String getName() {
        return name;
    }
}
