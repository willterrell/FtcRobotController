package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.Locale;

@TeleOp(name="Competition 3 TeleOp")
public class DraftTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;
    private final double carouselSpeed = 1;
    private ElapsedTime timer;
    private double prevTime;
    private final int slideSpeed = 250;
    private CRServo carousel;
    private boolean a, prevA, toggleA, back, prevBack, toggleBack = false;

    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        timer = new ElapsedTime();
        prevTime = timer.milliseconds();
    }

    @Override
    public void loop() {
        // drivetrain movement -> forward = left_stick_y, strafe = left_stick_x, rotate = right_stick_x
        a = gamepad1.a && !prevA;
        prevA = gamepad1.a;
        if (a) toggleA = !toggleA;

        back = gamepad1.back && !prevBack;
        prevBack = gamepad1.back;
        if (back) toggleBack = !toggleBack;

        double c = 0.5;
        if (toggleA) c = 1;
        double speed = (gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x) * c; // magnitude squared
        speed = Math.max(Math.min(1, speed), -1);
        hardwareHandler.moveWithPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);

        // carousel movement -> hold y
        double carouselIn;
        if (gamepad1.y) carouselIn = carouselSpeed;
        else if (gamepad1.x) carouselIn = -carouselSpeed;
        else carouselIn = 0;
        hardwareHandler.moveCarousel(carouselIn);

        // input spinner -> in = right_trigger, out = left_trigger
        double toggleInput = (toggleBack) ? 1 : 0;
        hardwareHandler.moveInputWheel(- gamepad1.right_trigger + gamepad1.left_trigger + gamepad2.right_trigger + toggleInput);

        // linear slide -> up = dpad_up, down = dpad_down
        hardwareHandler.addSlideSetpoint(150 * (gamepad1.dpad_up || gamepad1.left_bumper ? 1: 0) - 150 * (gamepad1.dpad_down || gamepad1.right_bumper ? 1: 0), !gamepad1.b);
        double[] lsTeles = hardwareHandler.updateSlides();
        telemetry.addData("LS Target Position:", hardwareHandler.getLSSetpoint());
        telemetry.addData("LS Inputs", String.format(Locale.ENGLISH, "%f, %f", lsTeles[0], lsTeles[1]));
        /*double pow = 0;
        if (gamepad1.dpad_up) pow = 0.5;
        if (gamepad1.dpad_down) pow = -0.5;
        hardwareHandler.simpleSlides(pow, pow);*/
        int[] lsPos = hardwareHandler.getLSEncoderPosition();
        telemetry.addData("LS Pos: ", String.format(Locale.ENGLISH, "{%d, %d}", lsPos[0], lsPos[1]));

        double[] dtPow = hardwareHandler.getPowers();
        double[] dtVel = hardwareHandler.getVelocities();
        telemetry.addData("Drivetrain power", String.format(Locale.ENGLISH, "{lf:%f, lr:%f, rf:%f, rr:%f}", dtVel[0], dtVel[1], dtVel[2], dtVel[3]));
        telemetry.addData("Drivetrain velocities", String.format(Locale.ENGLISH, "{lf:%f, lr:%f, rf:%f, rr:%f}", dtPow[0], dtPow[1], dtPow[2], dtPow[3]));
        telemetry.addData("Update time", timer.milliseconds() - prevTime);
        prevTime = timer.milliseconds();



    }
}
