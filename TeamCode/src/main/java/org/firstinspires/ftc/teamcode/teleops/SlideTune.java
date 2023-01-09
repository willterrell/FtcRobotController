package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.PIDFController;

@Config
@TeleOp(name="slideTuning2022")
public class SlideTune extends OpMode {
    private HardwareHandler hardwareHandler;
    public static double p = 0.007, i = 0, d = -0.005, g = 0.001;
    public static int target = 0;
    private PIDFController controller;
    private ElapsedTime timer;
    private double prevTime;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        controller = new PIDFController(p, i, d, 0, g, 0);
        timer = new ElapsedTime();
        telemetry.addData("target", target);
        telemetry.addData("currPos", 0);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.start) {
            controller.setCoefficientsAndReset(p, i, d, 0, g, 0);
        }
        double input = controller.getInput(target - hardwareHandler.getTargetSlidePos());
        hardwareHandler.setSlidePower(input);

        telemetry.addData("target", target);
        telemetry.addData("currPos", hardwareHandler.getTargetSlidePos());
        telemetry.addData("update", timer.milliseconds() - prevTime);
        telemetry.addData("p", controller.getP());
        telemetry.addData("i", controller.getI());
        telemetry.addData("d", controller.getD());
        prevTime = timer.milliseconds();
        telemetry.update();
    }
}
