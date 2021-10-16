package org.firstinspires.ftc.teamcode;

public class PlaceholderState extends AbState { // this state does nothing
    public PlaceholderState() {
        super("Placeholder");
    }

    @Override
    public void init() {

    }

    @Override
    public AbState next() {
        return this;
    }

    @Override
    public void run() {

    }
}
