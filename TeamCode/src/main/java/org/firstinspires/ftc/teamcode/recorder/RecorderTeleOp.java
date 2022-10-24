package org.firstinspires.ftc.teamcode.recorder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

import java.util.HashSet;

public class RecorderTeleOp extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean a, prevA = false, toggleA = false;
    private RecorderLog log;
    private final double ticksPerSecond = 20, secondsPerTick = 1000/ticksPerSecond;
    private double time = 0;
    private ElapsedTime timer;
    @Override
    public void init() { // TODO implement json serialization
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position());
        HashSet<String> alphabet = new HashSet<String>();
        alphabet.add("move");
        log = new RecorderLog(alphabet, ticksPerSecond);
        timer = new ElapsedTime(); // make sure this is zero at the beginning of loop
        /*Action move = new Action("move") {
            @Override
            public void execute(Object[] parameters) {
                Double[] doublePara;
                try {
                    doublePara = (Double[]) parameters;
                    // casting is probably the best solution
                }
                catch (ClassCastException e) {
                    doublePara = new Double[]{0.0, 0.0, 0.0, 0.0};
                    // add error throwing
                }
                hardwareHandler.moveWithPower(doublePara[0], doublePara[1], doublePara[2], doublePara[3]);
            }
        };*/
    }

    @Override
    public void loop() {
        if (timer.milliseconds() >= time) { // if statement to enforce a certain TPS
            // Regular TeleOp
            a = gamepad1.a && !prevA;
            prevA = gamepad1.a;
            if (a) toggleA = !toggleA;

            double c = 0.5;
            if (toggleA) c = 1;
            double speed = (gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x) * c; // magnitude squared
            speed = Math.max(Math.min(1, speed), -1);
            double f = gamepad1.left_stick_y, r = gamepad1.left_stick_x, s = gamepad1.right_stick_x;
            hardwareHandler.moveWithVelocity(f, r, s, speed);

            // Recording
            RecorderItem item = new RecorderItem(time);
            item.putParameter("move", new Double[]{f,r,s,speed});
            try {
                log.putItem(item);
            }
            catch (ActionNotIncluded e) {
                // add error telemetry
            }
            time = Math.round(timer.milliseconds());
            time += secondsPerTick - time % secondsPerTick; // rounds to nearest second per tick
        }
    }
}
