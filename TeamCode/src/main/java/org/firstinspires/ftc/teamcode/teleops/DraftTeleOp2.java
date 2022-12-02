package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

@Config
@TeleOp(name="draftTeleOp2")
public class DraftTeleOp2 extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean b, prevB = false, toggleB = false, a, prevA, toggleA, nearCone, prevNearCone;
    private boolean dpadDown, prevDpadDown = false, dpadUp, prevDpadUp = false;
    private Gamepad.RumbleEffect customRumble;
    private ElapsedTime timer;
    private double prevTime;
    public static double SLIDE_SPEED_COEFF = 0.3, LEFT_FRONT_COEFF = 1, LEFT_REAR_COEFF = 1, RIGHT_FRONT_COEFF = 1, RIGHT_REAR_COEFF = 1;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);

        customRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .build();

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        prevTime = timer.milliseconds();
        b = gamepad1.b && !prevB;
        prevB = gamepad1.b;
        if (b) toggleB = !toggleB;

        a = gamepad1.a && !prevA;
        prevA = gamepad1.a;
        if (a) toggleA = !toggleA;

        dpadUp = gamepad2.dpad_up && !prevDpadUp;
        prevDpadUp = gamepad2.dpad_up;

        dpadDown = gamepad2.dpad_down && !prevDpadDown;
        prevDpadDown = gamepad2.dpad_down;

        //boolean near = hardwareHandler.getClawSensorDistance(DistanceUnit.INCH) < 1.5;
        //nearCone = near && !prevNearCone;
        //prevNearCone = near;

        if (b && !toggleB) {
            hardwareHandler.openClaw();
        }
        if (b && toggleB) {
            hardwareHandler.closeClaw();
        }

        double c = 0.5;
        if (toggleA) c = 1;
        if (gamepad1.x) c = 0.1;
        double speed = Math.max(Math.max(gamepad1.left_stick_x * gamepad1.left_stick_x, 0.5 * gamepad1.left_stick_y * gamepad1.left_stick_y), gamepad1.right_stick_x * gamepad1.right_stick_x) * c; // magnitude squared
        speed = Math.max(Math.min(c, speed), 0);
        hardwareHandler.moveWithPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);
        telemetry.addData("left_stick_y", gamepad2.left_stick_y);
        //hardwareHandler.moveOneSlideWithPower(((gamepad1.dpad_up)? 1 : 0) + ((gamepad1.dpad_down)? -1 : 0));

        telemetry.addData("clawSensor", hardwareHandler.getClawSensorDistance(DistanceUnit.INCH));
        //telemetry.addData("near", near);
        telemetry.addData("rising", nearCone);
        telemetry.addData("t", timer.milliseconds() - prevTime);
        prevTime = timer.milliseconds();
//        if (nearCone) {
//            gamepad1.runRumbleEffect(customRumble);
//        }
        double slideSpeed = SLIDE_SPEED_COEFF * Math.signum(gamepad2.right_stick_y) * gamepad2.right_stick_y * gamepad2.right_stick_y;
        hardwareHandler.moveSlide(slideSpeed, !gamepad2.a, telemetry);
//        hardwareHandler.updateSlide(0.3);
        telemetry.addData("ls", slideSpeed);
        telemetry.addData("dpadUp", dpadUp);
        telemetry.addData("dpadDown", dpadDown);
    }
}
