package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PosType;

public class StateFactory {
    public static AbState getMoveState(String name, HardwareHandler hardwareHandler, Position pos, double angle, double speed, PosType posType) {
        return new EncoderMove(name, hardwareHandler, pos, angle, speed, posType);
    }
}
