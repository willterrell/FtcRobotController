package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo Test")
public class ServoTest extends OpMode {
    private CRServo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPower(1);
    }
}
