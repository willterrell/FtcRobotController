package org.firstinspires.ftc.teamcode.odometry;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class OdometryTracker {
    private double radiusOfRotation;
    private Position position;

    public OdometryTracker(double radiusOfRotation, Position initialPosition){
        this.radiusOfRotation = radiusOfRotation;
        this.position = initialPosition;
    }

    public void updatePosition(double wheel1, double wheel2, double wheel3) {
        // do this in the future: https://learnroadrunner.com/dead-wheels.html#two-wheel-odometry
        // we assume that wheel one and two are parallel, wheel three is orthogonal to the other two
        // wheel one is left of wheel two
        double forward = (wheel1 + wheel2) / 2;
        double rotation = Math.toDegrees((wheel1 - wheel2) / 2 / radiusOfRotation); // clockwise rotation
        double strafe = wheel3;
        double forceVectorRotation = Math.atan(strafe/forward);
        double distance = Math.sqrt(forward*forward + strafe*strafe);
        // derived from calculus, see desmos graph. I really don't know if this work
        double updatedX = position.x + (distance/rotation) * (Math.sin(rotation + forceVectorRotation) - Math.sin(forceVectorRotation));
        double updatedY = position.y + (distance/rotation) * (-Math.cos(rotation + forceVectorRotation) + Math.sin(forceVectorRotation));
        position = new Position(DistanceUnit.INCH, updatedX, updatedY, 0, 0);
    }
}
