package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.text.DecimalFormat;

@TeleOp(name="TeleOpDriveTune")
public class TeleOpDriveTune extends OpMode {
    private boolean rA, rB, rX, rY, rLeft, rRight, rBack, rStart, pA, pB, pX, pY, pLeft, pRight, pBack, pStart, running = false, up, down;
    private HardwareHandler hardwareHandler;
    private double kStrafeFront = 1, kStrafeBack = 1, diff = 1;
    private int typeIndex = 0, numTypes = 2;
    private String[] typeNames = new String[] {"kStrafeFront", "kStrafeBack"};
    private DecimalFormat form = new DecimalFormat("#.##########");
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position(), telemetry);
    }

    @Override
    public void loop() {
        // gathering inputs on the rising edge
        rA = gamepad1.a;
        rA = rA && ! pA;
        pA = gamepad1.a;

        rB = gamepad1.b;
        rB = rB && ! pB;
        pB = gamepad1.b;

        rX = gamepad1.x;
        rX = rX && ! pX;
        pX = gamepad1.x;

        rY = gamepad1.y;
        rY = rY && ! pY;
        pY = gamepad1.y;

        rLeft = gamepad1.dpad_left;
        rLeft = rLeft && !pLeft;
        pLeft = gamepad1.dpad_left;

        rRight = gamepad1.dpad_right;
        rRight = rRight && !pRight;
        pRight = gamepad1.dpad_right;

        rStart = gamepad1.start;
        rStart = rStart && ! pStart;
        pStart = gamepad1.start;

        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;

        //if (rStart && !running) init(p, i, d, iD, c, angle);

        //if (rStart && running) end();

        if (rStart) running = !running;

        if (running) run();

        else {
            if (rLeft || rRight) {
                typeIndex += (rLeft ? -1 : 0) + (rRight ? 1 : 0);
                typeIndex %= numTypes;
                if (typeIndex < 0) typeIndex += numTypes;
            }

            if (rX || rB) {
                diff *= (rX ? 2 : 1) * (rB ? 0.5 : 1);
            }

            if (rY || rA) {
                addTypeVal(typeIndex, diff * (rY ? 1 : 0) + diff * (rA ? -1 : 0));
                diff *= 0.5;
            }


            // you could omit parameters but could help with concurrency errors

            telemetry.addData("kFront: ", form.format(kStrafeFront));
            telemetry.addData("kBack: ", form.format(kStrafeBack));
            telemetry.addData("diff: ", form.format(diff));
            telemetry.addData("What you're changing:", typeNames[typeIndex]);
        }
    }

    private void run() {
        double c = 0.5;
        double speed = Math.max(Math.max(gamepad1.left_stick_x * gamepad1.left_stick_x, gamepad1.left_stick_y * gamepad1.left_stick_y), gamepad1.right_stick_x * gamepad1.right_stick_x) * c; // magnitude squared
        speed = Math.max(Math.min(c, speed), 0);
        hardwareHandler.moveWithPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speed);
    }

    private void addTypeVal(int typeIndex, double change) {
        if (typeIndex == 0) {
            kStrafeFront += change;
        }
        else {
            kStrafeBack += change;
        }
    }
}
