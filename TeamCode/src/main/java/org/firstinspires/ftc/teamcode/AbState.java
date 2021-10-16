package org.firstinspires.ftc.teamcode;

public abstract class AbState { // this is basically a decorator pattern
    protected String name;
    protected AbState currentState = this;

    public AbState(String name){
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public AbState getCurrentState(){
        return currentState;
    }

    public abstract void init(); // expected to be run in next

    public abstract AbState next(); // returns next state to be run and also end behavior

    public abstract void run();
}