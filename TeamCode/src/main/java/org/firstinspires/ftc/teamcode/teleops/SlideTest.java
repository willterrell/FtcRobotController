package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="slide test")
public class SlideTest extends OpMode {
    private DcMotor ls1, ls2;
    @Override
    public void init() {
        ls1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        ls2 = hardwareMap.get(DcMotor.class, "linearSlide2");
    }

    @Override
    public void loop() {
        ls1.setPower(gamepad1.left_stick_y);
        ls2.setPower(gamepad1.right_stick_y);
    }
}
