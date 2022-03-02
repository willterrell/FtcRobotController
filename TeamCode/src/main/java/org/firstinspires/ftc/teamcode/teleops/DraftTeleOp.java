package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.Locale;

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
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        timer = new ElapsedTime();
        prevTime = timer.milliseconds();
    }

    @Override
    public void loop() {
        // drivetrain movement -> forward = left_stick_y, strafe = left_stick_x, rotate = right_stick_x
        double c = 0.5;
        if (gamepad1.a) c = 1;
        else if (gamepad1.b) c = 0.5;
        double speed = (gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y) * c; // magnitude squared
        hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, 0, speed);

        // carousel movement -> hold y
        double carouselIn;
        if (gamepad1.y) carouselIn = carouselSpeed;
        else if (gamepad1.x) carouselIn = -carouselSpeed;
        else carouselIn = 0;
        hardwareHandler.moveCarousel(carouselIn);

        // input spinner -> in = right_trigger, out = left_trigger
        hardwareHandler.moveInputWheel(gamepad1.right_trigger - gamepad1.left_trigger);

        // linear slide -> up = dpad_up, down = dpad_down
        hardwareHandler.moveSlide(150 * (gamepad1.dpad_up ? 1: 0) - 150 * (gamepad1.dpad_down ? 1: 0));
        double[] lsTeles = hardwareHandler.updateSlides();
        telemetry.addData("LS Target Position:", hardwareHandler.getlSTargetPos());
        /*double pow = 0;
        if (gamepad1.dpad_up) pow = 0.5;
        if (gamepad1.dpad_down) pow = -0.5;
        hardwareHandler.simpleSlides(pow, pow);*/
        int[] lsPos = hardwareHandler.getLSPos();
        telemetry.addData("LS Pos: ", String.format(Locale.ENGLISH, "{%d, %d}", lsPos[0], lsPos[1]));

        double[] dtVel = hardwareHandler.getVelocities();
        telemetry.addData("Drivetrain velocities: ", String.format(Locale.ENGLISH, "{lf:%f, lr:%f, rf:%f, rr:%f}", dtVel[0], dtVel[1], dtVel[2], dtVel[3]));
    }
}
