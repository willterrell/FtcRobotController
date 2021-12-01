package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Draft 1")
public class DraftTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;
    private final double carouselSpeed = 0.5;
    private ElapsedTime timer;
    private double prevTime;
    private final double slideSpeed = 10;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
        timer = new ElapsedTime();
        prevTime = 0;
    }

    @Override
    public void loop() {
        // drivetrain movement -> forward = left_stick_y, strafe = left_stick_x, rotate = right_stick_x
        double speed = gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.right_stick_x*gamepad1.right_stick_x; // magnitude squared
        hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);

        // carousel movement -> hold y
        double carouselIn;
        if (gamepad1.y) carouselIn = carouselSpeed;
        else carouselIn = 0;
        hardwareHandler.moveCarousel(carouselIn);

        // input spinner -> in = right_trigger, out = left_trigger
        hardwareHandler.moveInputWheel(gamepad1.right_trigger - gamepad1.left_trigger);

        // linear slide -> up = dpad_up, down = dpad_down
        double dt = timer.milliseconds() - prevTime;
        double deltaPos = 0;
        if (gamepad1.dpad_up) deltaPos += 1;
        if (gamepad1.dpad_down) deltaPos -= 1;
        hardwareHandler.moveSlide((int) Math.round(deltaPos*slideSpeed), dt);
    }
}
