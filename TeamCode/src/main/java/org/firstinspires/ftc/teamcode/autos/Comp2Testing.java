package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

@Autonomous
@Config
public class Comp2Testing extends LinearOpMode {
    public static double START_X = 34.5, START_Y = 63, START_DEG = 180;
    private Pose2d START = new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    public static double MIDDLE_X = 34.5, MIDDLE_Y = 12, MIDDLE_DEG = 180;
    public static double HIGH_X = 24, HIGH_Y = 8.5, HIGH_DEG = -90;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        leftEncoder.reset();
        rightEncoder.reset();
        frontEncoder.reset();
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        drive.setPoseEstimateAndTrajEnd(START);
        Pose2d MIDDLE = new Pose2d(MIDDLE_X, MIDDLE_Y, Math.toRadians(MIDDLE_DEG));
        Trajectory traj1 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .lineToLinearHeading(MIDDLE)
                .build();
        Pose2d HIGH_JUNCTION = new Pose2d(HIGH_X, HIGH_Y, Math.toRadians(HIGH_DEG));
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(HIGH_JUNCTION)
                .build();
        MoveWithRoadrunner move1 = new MoveWithRoadrunner("startToMiddle", traj1, drive);
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("startToMiddle", traj2, drive);
        //move1.putNextState("next", move2);
        move1.putNextState("next", new PlaceholderState());
        AbState currState = move1;
        telemetry.addData("left", leftEncoder.getCurrentPosition());
        telemetry.addData("right", rightEncoder.getCurrentPosition());
        telemetry.addData("front", frontEncoder.getCurrentPosition());
        telemetry.update();
        currState.init();
        waitForStart();
        while (opModeIsActive()) {
            currState.run();
            currState = currState.next();
            drive.update();
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("front", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
