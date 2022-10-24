package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Deprecated
@TeleOp(name="Scoop Test", group="Examples")
public class ExamplePofC extends OpMode {
    private DcMotor myDcMotor;
    @Override
    public void init() {
        // myDcMotor = hardwareMap.get(DcMotor.class, "myDcMotor");
        myDcMotor = hardwareMap.dcMotor.get("scoop");
        myDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        myDcMotor.setPower(gamepad1.left_stick_y);
    }
}
