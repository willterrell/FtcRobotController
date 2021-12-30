package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp Draft 1")
public class DraftTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;
    private final double carouselSpeed = 1;
    private ElapsedTime timer;
    private double prevTime;
    private final int slideSpeed = 250;
    private CRServo carousel;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap);
        timer = new ElapsedTime();
        prevTime = timer.milliseconds();
    }

    @Override
    public void loop() {
        // drivetrain movement -> forward = left_stick_y, strafe = left_stick_x, rotate = right_stick_x
        double c = 0.5;
        if (gamepad1.a) c = 1;
        else if (gamepad1.b) c = 0.5;
        double speed = (gamepad1.left_stick_x*gamepad1.left_stick_x + gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.right_stick_x*gamepad1.right_stick_x) * c; // magnitude squared
        hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);

        // carousel movement -> hold y
        double carouselIn;
        if (gamepad1.y) carouselIn = carouselSpeed;
        else if (gamepad1.x) carouselIn = -carouselSpeed;
        else carouselIn = 0;
        hardwareHandler.moveCarousel(carouselIn);

        // input spinner -> in = right_trigger, out = left_trigger
        hardwareHandler.moveInputWheel(gamepad1.right_trigger - gamepad1.left_trigger);

        // linear slide -> up = dpad_up, down = dpad_down
        double dt = timer.milliseconds() - prevTime;
        int deltaPos = 0;
        if (gamepad1.dpad_up) deltaPos = 1;
        if (gamepad1.dpad_down) deltaPos = -1;
        hardwareHandler.moveSlide(deltaPos*slideSpeed);
        double[] lsTeles = hardwareHandler.updateSlides();
        double upIn = lsTeles[0];
        double diffIn = lsTeles[1];
        telemetry.addData("LS up in:", upIn);
        telemetry.addData("LS diff in:", upIn);
        telemetry.addData("LS Target Position:", hardwareHandler.getlSTargetPos());
        /*double pow = 0;
        if (gamepad1.dpad_up) pow = 0.5;
        if (gamepad1.dpad_down) pow = -0.5;
        hardwareHandler.simpleSlides(pow);*/

    }
}
