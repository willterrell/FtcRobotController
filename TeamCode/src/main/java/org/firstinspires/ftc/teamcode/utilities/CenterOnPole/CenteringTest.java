package org.firstinspires.ftc.teamcode.utilities.CenterOnPole;

import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.FindPole.DISTANCE;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.FindPole;

@Autonomous
@Config
public class CenteringTest extends LinearOpMode {
    public static boolean noCones = true;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        hardwareHandler.setDrive(drive);
        CenterOnPoleState center = new CenterOnPoleState("center", hardwareHandler, false, noCones);
        AbState currState = center;
        currState.init();
        telemetry.addData("init", timer.time());
        telemetry.addData("left", hardwareHandler.getRecordedLeftPoleSensor());
        telemetry.addData("right", hardwareHandler.getRecordedRightPoleSensor());
        telemetry.update();
        waitForStart();
        double prevTime = timer.milliseconds();
        while(opModeIsActive()) {
            currState.run();
            currState = currState.next();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("update", timer.milliseconds() - prevTime);
            prevTime = timer.milliseconds();
            if (hardwareHandler.getRecordedLeftPoleSensor() < 10) {
                telemetry.addData("left", hardwareHandler.getRecordedLeftPoleSensor());
                telemetry.addData("right", hardwareHandler.getRecordedRightPoleSensor());
            }
            telemetry.update();
            drive.update();
        }
    }
}
