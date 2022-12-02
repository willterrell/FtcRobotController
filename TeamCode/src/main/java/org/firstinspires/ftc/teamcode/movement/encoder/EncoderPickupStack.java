package org.firstinspires.ftc.teamcode.movement.encoder;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.HashMap;


public class EncoderPickupStack {
    private HardwareHandler hardwareHandler;
    private AbState currState;
    private double distanceReading = HardwareHandler.getDistanceSensorReading()+100;
    while distanceReading > 0 {
        distanceReading = HardwareHandler.getDistanceSensorReading();

    }

}
