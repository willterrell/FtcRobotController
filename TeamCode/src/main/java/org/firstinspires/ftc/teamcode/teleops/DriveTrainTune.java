package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.PIDType;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.text.DecimalFormat;


@TeleOp(name="DriveTrainTune")
public class DriveTrainTune extends OpMode {
    private boolean rA, rB, rX, rY, rLeft, rRight, rBack, rStart, pA, pB, pX, pY, pLeft, pRight, pBack, pStart, running = false, up, down; // r: rising, p: previous
    private PIDType currType = PIDType.P;
    private PIDType[] types = new PIDType[]{PIDType.P, PIDType.I, PIDType.D, PIDType.ID, PIDType.C};
    private int typeIndex = 0;
    private double diff = 1, p = 0.01, i = 0, d = 0.001, iD = 0, angle = 180, c = 0.0625; // 0.00658 0.0000015, 0.001
    private HardwareHandler hardwareHandler;
    private AbState currState, rotate;
    private DecimalFormat form = new DecimalFormat("#.##########");
    private final ElapsedTime timer = new ElapsedTime();
    private double time;
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

        rStart = gamepad1.start;
        rStart = rStart && ! pStart;
        pStart = gamepad1.start;

        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;

        if (rStart && !running) init(p, i, d, iD, c, angle);

        if (rStart && running) end();

        if (rStart) running = !running;

        if (running) run();

        else {
            if (up || down) angle += (up ? 0.1 : 0) + (down ? -0.1 : 0);

            if (rLeft || rRight) {
                typeIndex += (rLeft ? -1 : 0) + (rRight ? 1 : 0);
                typeIndex %= types.length;
                if (typeIndex < 0) typeIndex += types.length;
                currType = types[typeIndex];
            }

            if (rX || rB) {
                diff *= (rX ? 10 : 1) * (rB ? 0.1 : 1);
            }

            if (rY || rA) {
                addTypeVal(currType, diff * (rY ? 1 : 0) + diff * (rA ? -1 : 0));
            }

            // you could omit parameters but could help with concurrency errors

            telemetry.addData("p: ", form.format(p));
            telemetry.addData("i: ", form.format(i));
            telemetry.addData("d: ", form.format(d));
            telemetry.addData("iD: ", form.format(iD));
            telemetry.addData("c: ", form.format(c));
            telemetry.addData("", ""); // line break
            telemetry.addData("What you're changing:", "");
            telemetry.addData("angle: ", form.format(angle));
            telemetry.addData("s: ", form.format(diff));
            telemetry.addData(getTypeName(currType) + ": ", form.format(getTypeVal(currType)));
        }
    }

    private void end() {
        hardwareHandler.moveWithPower(0, 0, 0, 0);
    }

    private void init(double p, double i, double d, double iD, double c, double angle) {
        rotate = new RotateWithIMU("test rotate", hardwareHandler, angle, 0.3, p, i, d, iD, c);
        rotate.putNextState("next", new PlaceholderState());
        currState = rotate;
        currState.init();
        timer.reset();
        time = 0;
    }

    private void run() {
        currState.run();
        currState = currState.next();
        for (TelemetryObj obj : currState.getTelemetries()) {
            telemetry.addData(obj.getCaption(), obj.getContent());
        }
        telemetry.addData("time", time);
        if (!(currState instanceof PlaceholderState)) time = timer.seconds();
        telemetry.update();
    }
    
    private void addTypeVal(PIDType type, double add) {
        if (type == PIDType.P) p += add;
        else if (type == PIDType.I) i+= add;
        else if (type == PIDType.D) d += add;
        else if (type == PIDType.ID) iD += add;
        else c += add;
    }
    
    private String getTypeName(PIDType type) {
        if (type == PIDType.P) return "p";
        else if (type == PIDType.I) return "i";
        else if (type == PIDType.D) return "d";
        else if (type == PIDType.ID) return "iD";
        else return "c";
    }
    
    private double getTypeVal(PIDType type) {
        if (type == PIDType.P) return p;
        else if (type == PIDType.I) return i;
        else if (type == PIDType.D) return d;
        else if (type == PIDType.ID) return iD;
        else return c;
    }
}
