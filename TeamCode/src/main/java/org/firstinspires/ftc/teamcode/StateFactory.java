package org.firstinspires.ftc.teamcode;

public class StateFactory {
    private HardwareHandler hardwareHandler;
    private AbState prevState;
    public StateFactory(HardwareHandler hardwareHandler) { // makes it easier to string together states
        this.hardwareHandler = hardwareHandler;
    }
    // main problem: if we initiate in chronological order, states must have a function setNextState
    // this function must be defined in AbState due to polymorphism; however, a state might have multiple next states
    // if we call with a list of nextStates, StateFactory wouldn't know how many to input for setNextState
    // solution: StateFactory contains a HashMap, adder functions are given the name of nextStates?
    // in that case, functions will run and everything will be initiated at the end

}
