package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo Test", group="deprecated")
public class ServoTest extends OpMode {
    private CRServo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "carousel");
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void loop() {
        servo.setPower(1);
    }
}
