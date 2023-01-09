package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class EncoderTest extends OpMode {
    private StandardTrackingWheelLocalizer localizer;
    @Override
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        List<Double> pos = localizer.getWheelPositions();
        telemetry.addData("left", pos.get(0));
        telemetry.addData("right", pos.get(1));
        telemetry.addData("front", pos.get(2));
    }
}
