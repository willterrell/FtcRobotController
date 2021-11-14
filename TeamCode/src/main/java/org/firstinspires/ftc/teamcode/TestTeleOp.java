package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonSerializer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.checkerframework.checker.units.qual.Speed;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

@TeleOp(name="TestTeleOp", group="TeleOp")
public class TestTeleOp extends LinearOpMode {
    private HardwareHandler hardwareHandler;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("start", true);
        telemetry.update();
        hardwareHandler = new HardwareHandler(hardwareMap);
        telemetry.addData("handler init", true);
        telemetry.update();
        hardwareHandler.initIMU(new Position(), new Velocity());
        telemetry.addData("imu init", true);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("initialized:",true);

            double speed = (Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2))/2;
            hardwareHandler.move(gamepad1.left_stick_y, gamepad1.left_stick_x, speed);

            Position curPos = hardwareHandler.getIMUPosition();

            telemetry.addData("position", curPos);
            telemetry.addData("rotation:", hardwareHandler.getIMUZAngle());
            telemetry.addData("accel:", hardwareHandler.getIMUAccel());
            telemetry.update();

            if (gamepad1.a) {
                String filename = "IMUTranslation.json";
                File file = new File(filename);
                String serial = String.format(Locale.ENGLISH, "{\"x\":%f,\"y\":%f,\"z\":%f}", curPos.x, curPos.y, curPos.z);
                ReadWriteFile.writeFile(file, serial);
            }
        }
    }
}