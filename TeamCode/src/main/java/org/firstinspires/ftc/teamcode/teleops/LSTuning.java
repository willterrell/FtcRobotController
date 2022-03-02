package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.structures.LSType;

import java.io.File;
import java.util.Locale;

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
    private boolean prevX = false, prevY = false, prevA = false, prevB = false, toggleB = false, prevBump;
    private LSType type;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        file = new File("LSPIDCoeff.json");
        type = LSType.DIFF;
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

        boolean bump = gamepad1.right_bumper && !prevBump;
        prevBump = gamepad1.right_bumper;

        // disable other slides based on type

        if (bump) {
            if (type == LSType.UP_LEFT) type = LSType.UP_RIGHT;
            else if (type == LSType.UP_RIGHT) type = LSType.DIFF;
            else type = LSType.UP_LEFT;
        }

        if (type == LSType.UP_LEFT) {
            hardwareHandler.disableLS(LSType.UP_LEFT, true);
            hardwareHandler.disableLS(LSType.UP_RIGHT, false);
            hardwareHandler.disableLS(LSType.DIFF, false);
        }
        else if (type == LSType.UP_RIGHT) {
            hardwareHandler.disableLS(LSType.UP_LEFT, false);
            hardwareHandler.disableLS(LSType.UP_RIGHT, true);
            hardwareHandler.disableLS(LSType.DIFF, false);
        }
        else {
            hardwareHandler.disableLS(LSType.UP_LEFT, true);
            hardwareHandler.disableLS(LSType.UP_RIGHT, true);
            hardwareHandler.disableLS(LSType.DIFF, true);
        }


        double[] tele = hardwareHandler.updateSlides();
        hardwareHandler.moveSlide(200 * (gamepad1.dpad_up ? 1: 0) - 200 * (gamepad1.dpad_down ? 1: 0)); // ? is ternary operator (basically if statement)

        double[] coeff = hardwareHandler.getLSPIDCoeff(type);

        if (toggleB) { // binary search coeff, second part
            double diff = Math.abs(coeff[0] - prevP);
            if (x) {
                hardwareHandler.setLSPIDCoeff(coeff[0] - 0.5 * diff, coeff[1], coeff[2], type);
                prevP = coeff[0];
            }

            else if (y) {
                hardwareHandler.setLSPIDCoeff(coeff[0] + 0.5 * diff, coeff[1], coeff[2], type);
                prevP = coeff[0];
            }
        }

        else { // halve or double p coefficient, first part
            if (x) {
                hardwareHandler.setLSPIDCoeff(0.5 * coeff[0], coeff[1], coeff[2], type);
                prevP = coeff[0];
            }
            else if (y) {
                hardwareHandler.setLSPIDCoeff(2.0 * coeff[0], coeff[1], coeff[2], type);
                prevP = coeff[0];
            }
        }

        telemetry.addData("PID P: ", coeff[0]);
        int[] lsPos = hardwareHandler.getLSPos();
        telemetry.addData("LS Pos: ", String.format(Locale.ENGLISH, "{%d, %d}", lsPos[0], lsPos[1]));
        telemetry.addData("LS left in:", tele[0]);
        telemetry.addData("LS right in:", tele[1]);
        telemetry.addData("LS Target Position:", hardwareHandler.getlSTargetPos());

        /*if (a) { // serialize coefficient to object
            JSONObject json = new JSONObject();
            try {
                json.put("upKP", coeff[0]).put("upKI", coeff[1]).put("upKD", 0);
            } catch (JSONException e) {
                e.printStackTrace();
            }
            ReadWriteFile.writeFile(file, json.toString());
        }*/

    }
}
