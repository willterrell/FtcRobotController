package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.Arrays;

@Autonomous
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            double[] color = hardwareHandler.getColorSensorReading();
            telemetry.addData("color", Arrays.toString(color));
            telemetry.update();
        }
    }
}
