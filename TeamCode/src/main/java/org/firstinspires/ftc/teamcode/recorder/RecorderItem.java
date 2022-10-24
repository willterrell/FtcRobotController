package org.firstinspires.ftc.teamcode.recorder;

import java.util.Map;
import java.util.Set;

public class RecorderItem {
    private Map<String, Object[]> parameters;
    private transient double time;

    public RecorderItem(double timeMS) {
        time = timeMS;
    }

    public void putParameter(String name, Object[] values) {
        parameters.put(name, values);
    }
    public Object[] getParameter(String name) {
        return parameters.get(name);
    }
    public double getTimeMS() {
        return time;
    }
    public Set<String> getNames() {
        return parameters.keySet();
    }
}
