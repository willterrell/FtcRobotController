package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.util.concurrent.locks.ReadWriteLock;

/*
In general, the best way to tune it will to be to halve the input (x) if it overshoots or double it
(y) if it doesn't, until it gets to the threshold.

Then, binary search the exact threshold: x+b for overshoot, y+b for not
 */

@TeleOp(name="Linear Slide Tuning")
public class LSTuning extends OpMode {
    private HardwareHandler hardwareHandler;
    private File file;
    private double prevP = 0;
    private boolean prevX = false, prevY = false, prevA = false, prevB = false, toggleB = false;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        file = new File("LSPIDCoeff.json");
    }

    @Override
    public void loop() {
        boolean x = gamepad1.x && !prevX; // ensures its on the rising edge
        prevX = gamepad1.x;

        boolean y = gamepad1.y && !prevY;
        prevY = gamepad1.y;

        boolean a = gamepad1.a && !prevA;
        prevA = gamepad1.a;

        boolean b = gamepad1.b && !prevB;
        prevB = gamepad1.b;
        if (b) toggleB = !toggleB; // toggles b


        hardwareHandler.updateSlides();
        hardwareHandler.moveSlide(100 * (gamepad1.dpad_up ? 1: 0)); // ? is ternary operator (basically if statement)

        double[] coeff = hardwareHandler.getUpLSPIDCoeff();

        if (toggleB) { // binary search coeff, second part
            double diff = Math.abs(coeff[0] - prevP);
            if (x) {
                hardwareHandler.setUpLSPIDCoeff(coeff[0] - 0.5 * diff, coeff[1], coeff[2]);
            }

            else if (y) {
                hardwareHandler.setUpLSPIDCoeff(coeff[0] + 0.5 * diff, coeff[1], coeff[2]);
            }
        }

        else { // halve or double p coefficient, first part
            hardwareHandler.setUpLSPIDCoeff(0.5 * (x ? 1: 0) * coeff[0], coeff[1], coeff[2]);
            hardwareHandler.setUpLSPIDCoeff(2 * (y ? 1: 0) * coeff[0], coeff[1], coeff[2]);
        }

        telemetry.addData("PID Coefficients: ", coeff);

        if (a) { // serialize coefficient to object
            JSONObject json = new JSONObject();
            try {
                json.put("upKP", coeff[0]).put("upKI", coeff[1]).put("upKD", 0);
            } catch (JSONException e) {
                e.printStackTrace();
            }
            ReadWriteFile.writeFile(file, json.toString());
        }

        prevP = coeff[0];
    }
}
