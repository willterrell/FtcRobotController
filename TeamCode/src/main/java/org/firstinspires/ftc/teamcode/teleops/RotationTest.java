package org.firstinspires.ftc.teamcode.teleops;

import android.graphics.LinearGradient;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Config
@TeleOp(name="rotationTest2022")
public class RotationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        AbState currState;
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.start) {
                currState = new RotateWithIMU("rotate", hardwareHandler);
                currState.putNextState("next", new PlaceholderState());
                currState.init();
                double timeStart = timer.milliseconds();
                double prevTime = timeStart;
                while (!(currState instanceof PlaceholderState)) {
                    currState.run();
                    currState = currState.next();

                    for (TelemetryObj tele : currState.getTelemetries()) {
                        dashboardTelemetry.addData(tele.getCaption(), tele.getContent());
                    }
                    dashboardTelemetry.update();

                    if (gamepad1.back) break;
                    dashboardTelemetry.addData("update", timer.milliseconds() - prevTime);
                    prevTime = timer.milliseconds();
                }
                dashboardTelemetry.addData("t", timer.milliseconds() - timeStart);
                dashboardTelemetry.update();
                hardwareHandler.moveWithPower(0, 0, 0, 0);
            }
        }
    }
}
