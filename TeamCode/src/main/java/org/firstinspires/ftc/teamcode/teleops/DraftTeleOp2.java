package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState;

@Config
@TeleOp(name="draftTeleOp2")
public class DraftTeleOp2 extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean b, prevB = false, toggleB = false, a, prevA, toggleA, risingNearCone, prevNearCone;
    private boolean x2, prevX2 = false, b2, prevB2 = false, a2, y2, prevA2 = false, prevY2 = false;
    private boolean dpadDown, prevDpadDown = false, dpadUp, prevDpadUp = false;
    private boolean bumpers, prevBumpers = false, autoRotate = false, triggers, prevTriggers = false, lockOn = false;
    private boolean y, prevY = false;
    private Gamepad.RumbleEffect customRumble;
    private ElapsedTime timer;
    private double prevTime;
    public static double SLIDE_SPEED_COEFF = 0.3, LEFT_FRONT_COEFF = 1, LEFT_REAR_COEFF = 1, RIGHT_FRONT_COEFF = 1, RIGHT_REAR_COEFF = 1;
    private double nearest90 = 0;
    private AbState currState;
    private SampleMecanumDrive drive;
    public static double A_COEFF = 0.8;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        hardwareHandler.setDrive(drive);
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

        x2 = gamepad2.x && !prevX2;
        prevX2 = gamepad2.x;

        b2 = gamepad2.b && !prevB2;
        prevB2 = gamepad2.b;

        a2 = gamepad2.a && !prevA2;
        prevA2 = gamepad2.a;

        y2 = gamepad2.y && !prevY2;
        prevY2 = gamepad2.y;

        bumpers = gamepad1.left_bumper || gamepad1.right_bumper && !prevBumpers;
        prevBumpers = gamepad1.left_bumper || gamepad1.right_bumper;

        triggers = gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5 && !prevTriggers;
        prevTriggers = gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5;

        y = gamepad1.y && !prevY;
        prevY = gamepad1.y;

//        boolean nearCone = hardwareHandler.getClawSensorDistance(DistanceUnit.INCH) < 1.5;
//        risingNearCone = nearCone && !prevNearCone;
//        prevNearCone = nearCone;

        if (b && !toggleB) {
            hardwareHandler.openClaw();
        }
        if (b && toggleB) {
            hardwareHandler.closeClaw();
        }

        double f = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x * 0.5 / 0.65;
        double s = gamepad1.left_stick_x;

        boolean manualMove = f != 0 || r != 0 || s != 0;

        telemetry.addData("angle", hardwareHandler.getIMUZAngle());
        telemetry.addData("nearest90", nearest90);

        if (bumpers || y) {
            if (bumpers) nearest90 = hardwareHandler.getNearest90AboveOrBelow(gamepad1.right_bumper); // turning with bumpers
            else nearest90 = hardwareHandler.getNearest90(); // aligning with y
            currState = new RotateWithIMU("autorotate", hardwareHandler, nearest90 - hardwareHandler.getIMUZAngle());
            currState.putNextState("next", new PlaceholderState());
            currState.init();
            autoRotate = true;
        }
        if (triggers) {
            drive.setPoseEstimateAndTrajEnd(new Pose2d());
            currState = new CenterOnPoleState("autorotate", hardwareHandler, false, gamepad1.left_trigger > 0.5);
            currState.putNextState("next", new PlaceholderState());
            currState.init();
            lockOn = true;
        }
        if (manualMove) {
            autoRotate = false;
            lockOn = false;
        }
        if (autoRotate || lockOn) {
            currState.run();
            currState = currState.next();
            drive.update();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            if (currState instanceof PlaceholderState) {
                manualMove = true;
                gamepad1.runRumbleEffect(customRumble);
            }
        }
        else {
            double c = 0.65;
            if (toggleA) c = A_COEFF;
            if (gamepad1.x) c = 0.2;
            double speed = Math.max(Math.max(f * f, r * r), s * s) * c; // magnitude squared
            speed = Math.max(Math.min(c, speed), 0);
            hardwareHandler.moveWithPower(f, r, s, speed);
        }


        //



//        telemetry.addData("left_stick_y", gamepad2.left_stick_y);
        //hardwareHandler.moveOneSlideWithPower(((gamepad1.dpad_up)? 1 : 0) + ((gamepad1.dpad_down)? -1 : 0));

//        telemetry.addData("clawSensor", hardwareHandler.getClawSensorDistance(DistanceUnit.INCH));
        //telemetry.addData("near", near);
        telemetry.addData("rising", risingNearCone);
        if (risingNearCone) {
            hardwareHandler.closeClaw();
            toggleB = false;
        }
//        double slideSpeed = SLIDE_SPEED_COEFF * Math.signum(gamepad2.right_stick_y) * gamepad2.right_stick_y * gamepad2.right_stick_y;
//        hardwareHandler.moveSlide(slideSpeed, !gamepad2.a, telemetry);
//        hardwareHandler.updateSlide(0.3);
//        hardwareHandler.shiftSlidePosition(dpadUp, dpadDown, x2, b2);
//        hardwareHandler.updateSlides();
        hardwareHandler.moveSlidesHybrid(0.3 * gamepad2.left_stick_y, dpadUp, dpadDown, !gamepad2.a, y2, x2, a2, b2);
//        telemetry.addData("dpadUp", dpadUp);
//        telemetry.addData("dpadDown", dpadDown);

        telemetry.addData("left", hardwareHandler.getLeftPoleSensor(DistanceUnit.INCH));

        telemetry.addData("t", timer.milliseconds() - prevTime);
        prevTime = timer.milliseconds();

        telemetry.update();
    }
}
