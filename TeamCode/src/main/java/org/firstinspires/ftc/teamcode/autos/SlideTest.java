package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;

@Autonomous(name="SlideTest")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        waitForStart();
        hardwareHandler.moveSlidesToPreset(SlidePosition.TWO_CONE);
        while(opModeIsActive()) {

        }
    }
}
