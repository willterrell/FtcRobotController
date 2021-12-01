package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Example Proof of Concept", group="Examples")
public class ExamplePofC extends OpMode {
    private DcMotor myDcMotor;
    @Override
    public void init() {
        // myDcMotor = hardwareMap.get(DcMotor.class, "myDcMotor");
        myDcMotor = hardwareMap.dcMotor.get("myDcMotor");
        myDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myDcMotor.setDirection(DcMotorSimple.Direction.FORWARD); // this is just so you don't have to write -1 every time you do set power
    }

    @Override
    public void loop() {
        myDcMotor.setPower(gamepad1.left_trigger);
        if (gamepad1.a) {
            myDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        telemetry.addData("gamepad1 left-trigger: ", gamepad1.left_trigger);
        telemetry.update();


    }
}
