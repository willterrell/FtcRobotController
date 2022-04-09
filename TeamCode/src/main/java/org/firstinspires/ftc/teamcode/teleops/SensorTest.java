package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name="Sensor test")
public class SensorTest extends OpMode {
    private DistanceSensor distanceSensor;
    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "barcode");
    }

    @Override
    public void loop() {
        telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
