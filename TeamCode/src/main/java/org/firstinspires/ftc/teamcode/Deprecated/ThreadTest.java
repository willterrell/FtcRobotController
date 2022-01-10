package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.movement.imu.SimpsonIntegrator;

@Deprecated
class Main implements Runnable {
    private SimpsonIntegrator simp;

    public Main(SimpsonIntegrator integrator) {
        simp = integrator;

    }


    @Override
    public void run() {
        simp.update(new Acceleration(DistanceUnit.CM, 1, 1, 1, 0));
    }
}
@Deprecated
@TeleOp(name="deprecated compTest", group="deprecated")
public class ThreadTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new SimpsonIntegrator(1000);

        SimpsonIntegrator integ = new SimpsonIntegrator(1000);

        integ.initialize(parameters, null, null);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        double mil = 0;
        while (opModeIsActive()) {
            mil = timer.milliseconds();
            telemetry.addData("in loop: ", true);
            integ.update(new Acceleration(DistanceUnit.CM, 1, 1, 1, 0));
            double nMil = timer.milliseconds();
            telemetry.addData("dt: ", nMil - mil);
            telemetry.update();
        }
    }
}
