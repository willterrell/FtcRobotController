package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.security.KeyException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;

public class TelemetryFactory {
    private final static ArrayList<TelemetryObj> teleArr = new ArrayList<TelemetryObj>();

    public static void add(TelemetryObj obj) {
        teleArr.add(obj);
    }

    public static ArrayList<TelemetryObj> getTelemetries() {
        return teleArr;
    }
}
