package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.structures.PIDType;

public class PIDTuneTemplate extends OpMode {
    private boolean rA, rB, rX, rY, rLBump, rRBump, rBack, rStart, pA, pB, pX, pY, pLBump, pRBump, pBack, pStart; // r: rising, p: previous
    private PIDType currType = PIDType.P;
    private PIDType[] types = new PIDType[]{PIDType.P, PIDType.I, PIDType.D};
    private int typeIndex = 0;
    private double diff = 1, p, i, d;
    @Override
    public void init() {
        
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

        rLBump = gamepad1.left_bumper;
        rLBump = rLBump && ! pLBump;
        pLBump = gamepad1.left_bumper;

        rRBump = gamepad1.left_bumper;
        rRBump = rRBump && ! pRBump;
        pRBump = gamepad1.right_bumper;

        rStart = gamepad1.start;
        rStart = rStart && ! pStart;
        pStart = gamepad1.start;


        
        if (rLBump || rRBump) {
            typeIndex += (rLBump ? -1 : 0) + (rRBump ? 1 : 0);
            typeIndex %= 3;
            currType = types[typeIndex];
        }

        if (rX || rB) {
            diff *= (rX ? 10 : 1) * (rB ? 0.1 : 1);
        }

        if (rY || rA) {
            addTypeVal(currType, diff * (rY ? 1 : 0) + diff * (rA ? -1 : 0));
        }

        if (rStart) run(p, i, d); // you could omit parameters but could help with concurrency errors

        telemetry.addData("p: ", p);
        telemetry.addData("i: ", i);
        telemetry.addData("d: ", d);
        telemetry.addData("", ""); // line break
        telemetry.addData("What you're changing:", "");
        telemetry.addData("diff: ", diff);
        telemetry.addData(getTypeName(currType) + ": ", getTypeVal(currType));
    }

    private void run(double p, double i, double d) {
        while (!rBack) {
            rBack = gamepad1.back;
            rBack = rBack && ! pBack;
            pBack = gamepad1.back;

            /*
            PUT CODE HERE
             */


        }
    }

    private void addTypeVal(PIDType type, double add) {
        if (type == PIDType.P) p += add;
        else if (type == PIDType.I) i+= add;
        else d += add;
    }

    private String getTypeName(PIDType type) {
        if (type == PIDType.P) return "p";
        else if (type == PIDType.I) return "i";
        else return "d";
    }

    private double getTypeVal(PIDType type) {
        if (type == PIDType.P) return p;
        else if (type == PIDType.I) return i;
        else return d;
    }
}
