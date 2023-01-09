package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        SampleMecanumDrive drive = hardwareHandler.getDrive();
        Trajectory traj;
        drive.setPoseEstimateAndTrajEnd(new Pose2d(5, 0));

        traj = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .strafeLeft(10)
                .build();
        MoveWithRoadrunner move = new MoveWithRoadrunner("backAndForth", traj, drive);
        move.putNextState("next", new PlaceholderState());
        AbState currState = move;
        currState.init();
        waitForStart();
        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.update();
        }
    }
}
