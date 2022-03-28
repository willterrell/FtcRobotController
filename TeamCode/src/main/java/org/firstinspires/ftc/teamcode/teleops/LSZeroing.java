package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.HardwareHandler;

@TeleOp(name="LS Zeroing")
public class LSZeroing extends OpMode {
    private HardwareHandler hardwareHandler;
    private boolean toggleB = false, prevB = false, prevA = false;
    @Override
    public void init() {
        hardwareHandler = new HardwareHandler(hardwareMap, new Position());
    }

    @Override
    public void loop() {
        boolean b = gamepad1.b && !prevB;
        prevB = gamepad1.b;
        if (b) toggleB = !toggleB; // toggles b

        boolean a = gamepad1.a && !prevA;
        prevA = gamepad1.a;

        double pow = 0;
        if (gamepad1.dpad_up) pow = 0.5;
        if (gamepad1.dpad_down) pow = -0.5;

        if (toggleB) {
            hardwareHandler.moveSlidesWithPower(0, pow);
        }
        else {
            hardwareHandler.moveSlidesWithPower(pow, 0);
        }

        if (a) {
            //hardwareHandler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
