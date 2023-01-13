package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.movement.roadrunner.HighJunctionToParking;
import org.firstinspires.ftc.teamcode.movement.roadrunner.StartToHighJunction;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.ParkingPosition;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState;
import org.firstinspires.ftc.teamcode.utilities.OpenClawState;

@Autonomous
@Config
public class Comp2Auto extends LinearOpMode {
    public static double START_X = 39.5, START_Y = 63, START_DEG = 180;
    public static Pose2d START = new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    public static boolean TEST = true;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, new Position(), START_DEG, telemetry);
        hardwareHandler.setDrive(drive);
        drive.setPoseEstimateAndTrajEnd(START);
        StartToHighJunction startToHighJunction = new StartToHighJunction("high", hardwareHandler);
        CenterOnPoleState center1 = new CenterOnPoleState("center1", hardwareHandler, true, true);
        OpenClawState open = new OpenClawState("open", hardwareHandler);
        HighJunctionToParking parking = new HighJunctionToParking("parking", hardwareHandler, ParkingPosition.ONE);
        RotateWithIMU finalRotate = new RotateWithIMU("align", hardwareHandler, 0, PosType.ABSOLUTE);
//        HighJunctionToConeStack highJunctionToConeStack = new HighJunctionToConeStack("coneStack", drive);
//        CenterOnPoleState center2 = new CenterOnPoleState("center2", hardwareHandler, false, true);
//        ConeStackToHighJunction coneStackToHighJunction = new ConeStackToHighJunction("highJunction2", drive);
//        CenterOnPoleState center3 = new CenterOnPoleState("center3", hardwareHandler, true, true);
        startToHighJunction.putNextState("next", center1);
        center1.putNextState("next", open);
        open.putNextState("next", parking);
        parking.putNextState("next", finalRotate);
        finalRotate.putNextState("next", new PlaceholderState());

//        center1.putNextState("error", new PlaceholderState());
//        highJunctionToConeStack.putNextState("next", center2);
//        center2.putNextState("next", coneStackToHighJunction);
//        coneStackToHighJunction.putNextState("next", center3);
//        center3.putNextState("next", new PlaceholderState());
        AbState currState = startToHighJunction;
        currState.init();
        waitForStart();
        double prevTime = timer.milliseconds();
        hardwareHandler.closeClaw();
        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
            drive.update();
            hardwareHandler.updateSlides();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.addData("r", hardwareHandler.getIMUZAngle());
            telemetry.addData("update", timer.milliseconds() - prevTime);
            prevTime = timer.milliseconds();
            telemetry.update();
        }
    }
}
