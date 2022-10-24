package org.firstinspires.ftc.teamcode.recorder;

public class ItemNotFound extends Exception {
    private String name;
    public ItemNotFound(String name) {
        this.name = name;
    }

    @Override
    public String toString() {
        return name + "not found in items";
    }
}
