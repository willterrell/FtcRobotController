package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.LSType;
import org.firstinspires.ftc.teamcode.structures.PIDType;

import java.text.DecimalFormat;
import java.util.Locale;

@TeleOp(name="Linear Slide Tuning")
public class LSTuning extends OpMode {
    private boolean rA, rB, rX, rY, rLeft, rRight, rBack, rStart, pA, pB, pX, pY, pLeft, pRight, pBack, pStart, running = false, up, down; // r: rising, p: previous
    private boolean rUp, pUp, rDown, pDown;
    private PIDType[] coeffTypes = new PIDType[]{PIDType.P, PIDType.I, PIDType.D, PIDType.ID, PIDType.C};
    private LSType[] lsTypes = new LSType[]{LSType.UP_LEFT, LSType.UP_RIGHT, LSType.DIFF};
    private double[][] coeffs = new double[lsTypes.length][coeffTypes.length];
    private double diff = 1;
    private int coeffIndex = 0, lsIndex = 0;
    private HardwareHandler hardwareHandler;
    private DecimalFormat form = new DecimalFormat("#.##########");
    private final ElapsedTime timer = new ElapsedTime();
    private double time; // 0.0015 TODO add dampening
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
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

        rUp = gamepad1.dpad_up;
        rUp = rUp && !pUp;
        pUp = gamepad1.dpad_up;

        rDown = gamepad1.dpad_down;
        rDown = rDown && !pDown;
        pDown = gamepad1.dpad_down;

        rStart = gamepad1.start;
        rStart = rStart && ! pStart;
        pStart = gamepad1.start;

        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;

        if (rStart && !running) initRun();

        if (rStart && running) end();

        if (rStart) running = !running;

        if (running) run();

        else {
            if (rLeft || rRight) {
                coeffIndex += (rLeft ? -1 : 0) + (rRight ? 1 : 0);
                coeffIndex %= coeffTypes.length;
                if (coeffIndex < 0) coeffIndex += coeffTypes.length;
            }

            if (rUp || rDown) {
                lsIndex += (rUp ? -1 : 0) + (rDown ? 1 : 0);
                lsIndex %= lsTypes.length;
                if (lsIndex < 0) lsIndex += lsTypes.length;
            }

            if (rX || rB) {
                diff *= (rX ? 10 : 1) * (rB ? 0.1 : 1);
            }

            if (rY || rA) {
                addTypeVal(lsIndex, coeffIndex, diff * (rY ? 1 : 0) + diff * (rA ? -1 : 0));
            }

            hardwareHandler.resetSlidePosition();

            // you could omit parameters but could help with concurrency errors
            telemetry.addData("PID Type", getLSTypeName(lsIndex));
            for (int i = 0; i < coeffs[lsIndex].length; i++) { // adds telemetry for each coefficient (P, I, D, C)
                telemetry.addData(getCoeffTypeName(i), coeffs[lsIndex][i]);
            }
            telemetry.addData("", ""); // line break
            telemetry.addData("What you're changing:", "");
            telemetry.addData("s: ", form.format(diff));
            telemetry.addData(getTypeName(lsIndex, coeffIndex) + ": ", form.format(getTypeVal(lsIndex, coeffIndex)));
        }
    }

    private void end() {
        hardwareHandler.moveSlidesWithPower(0,0);
    }

    private void initRun() {
        if (lsIndex == 0) {
            hardwareHandler.disableLS(LSType.UP_LEFT, true);
            hardwareHandler.disableLS(LSType.UP_RIGHT, false);
            hardwareHandler.disableLS(LSType.DIFF, false);
        }
        else if (lsIndex == 1) {
            hardwareHandler.disableLS(LSType.UP_LEFT, false);
            hardwareHandler.disableLS(LSType.UP_RIGHT, true);
            hardwareHandler.disableLS(LSType.DIFF, false);
        }
        else {
            hardwareHandler.disableLS(LSType.UP_LEFT, true);
            hardwareHandler.disableLS(LSType.UP_RIGHT, true);
            hardwareHandler.disableLS(LSType.DIFF, true);
        }
        hardwareHandler.setLSPIDCoeff(coeffs[lsIndex][0],coeffs[lsIndex][1],coeffs[lsIndex][2], coeffs[lsIndex][3], coeffs[lsIndex][4],lsTypes[lsIndex]);
        hardwareHandler.setSlideSetpoint(2700);
    }

    private void run() {
        int[] positions = hardwareHandler.getLSEncoderPosition();
        hardwareHandler.updateSlides();
        telemetry.addData("Left Position", positions[0]);
        telemetry.addData("Right Position", positions[1]);
        telemetry.addData("Target", 2700);
    }

    private void addTypeVal(int lsIndex, int coeffIndex, double add) {
        coeffs[lsIndex][coeffIndex] += add;
    }

    private String getTypeName(int lsIndex, int coeffIndex) {
        return String.format(Locale.ENGLISH, "(%s,%s)", getLSTypeName(lsIndex),getCoeffTypeName(coeffIndex));
    }

    private String getLSTypeName(int lsIndex) {
        LSType lsType = lsTypes[lsIndex];
        return lsType.toString();
    }

    private String getCoeffTypeName(int coeffIndex) {
        PIDType coeffType = coeffTypes[coeffIndex];
        return coeffType.toString();
    }

    private double getTypeVal(int lsIndex, int coeffIndex) {
        return coeffs[lsIndex][coeffIndex];
    }
}
