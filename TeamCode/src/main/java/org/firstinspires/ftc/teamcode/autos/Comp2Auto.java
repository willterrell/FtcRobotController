package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.roadrunner.ConeStackToHighJunction;
import org.firstinspires.ftc.teamcode.movement.roadrunner.HighJunctionToConeStack;
import org.firstinspires.ftc.teamcode.movement.roadrunner.StartToHighJunction;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState;

@Autonomous
@Config
public class Comp2Auto extends LinearOpMode {
    public static double START_X = 34.5, START_Y = 63, START_DEG = 180;
    public static Pose2d START = new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        drive.setPoseEstimateAndTrajEnd(START);
        StartToHighJunction startToHighJunction = new StartToHighJunction("high", drive);
        CenterOnPoleState center1 = new CenterOnPoleState("center1", hardwareHandler, true, true);
        HighJunctionToConeStack highJunctionToConeStack = new HighJunctionToConeStack("coneStack", drive);
        CenterOnPoleState center2 = new CenterOnPoleState("center2", hardwareHandler, false, true);
        ConeStackToHighJunction coneStackToHighJunction = new ConeStackToHighJunction("highJunction2", drive);
        CenterOnPoleState center3 = new CenterOnPoleState("center3", hardwareHandler, true, true);
        startToHighJunction.putNextState("next", center1);
        center1.putNextState("next", highJunctionToConeStack);
        center1.putNextState("error", new PlaceholderState());
        highJunctionToConeStack.putNextState("next", center2);
        center2.putNextState("next", coneStackToHighJunction);
        coneStackToHighJunction.putNextState("next", center3);
        center3.putNextState("next", new PlaceholderState());
        AbState currState = startToHighJunction;
        currState.init();
        waitForStart();
        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("pose", drive.getPoseEstimate());
            telemetry.update();
            drive.update();
        }
    }
}
