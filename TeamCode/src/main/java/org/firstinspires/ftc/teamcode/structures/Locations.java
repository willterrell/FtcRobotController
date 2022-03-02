package org.firstinspires.ftc.teamcode.structures;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public enum Locations {
    CAROUSEL_START(new Position(DistanceUnit.INCH, 70.125, -35.0625, 0, 0)),
    HUB_START(new Position(DistanceUnit.INCH, 70.125, 11.6875, 0, 0)),
    CAROUSEL_BOTTOM_BARCODE(new Position(DistanceUnit.INCH, 35.063, -44.375, 0, 0)),
    CAROUSEL_MIDDLE_BARCODE(new Position(DistanceUnit.INCH, 35.063, -35.995, 0, 0)),
    CAROUSEL_TOP_BARCODE(new Position(DistanceUnit.INCH, 35.063, -27.615, 0, 0)),
    HUB_BOTTOM_BARCODE(new Position(DistanceUnit.INCH, 35.063, 2.875, 0, 0)),
    HUB_MIDDLE_BARCODE(new Position(DistanceUnit.INCH, 35.063, 11.255, 0, 0)),
    HUB_TOP_BARCODE(new Position(DistanceUnit.INCH, 35.063, 19.635, 0, 0)),
    SIDE_HUB(new Position(DistanceUnit.INCH, 23.375, -11.688, 0, 0)),
    BLOCK_DEPOT_MIDDLE(new Position(DistanceUnit.INCH, 50, 50, 0, 0));


    private final Position pos;
    Locations(Position pos){
        this.pos = pos;
    }
    public Position getPos(Side side){
        if (side == Side.BLUE) {
            return pos;
        }
        return new Position(DistanceUnit.INCH, -1 * pos.x, pos.y, 0, 0);
    }

}
