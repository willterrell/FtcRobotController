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
import org.firstinspires.ftc.teamcode.movement.roadrunner.HighJunctionToConeStack;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.SlidePosition;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;
import org.firstinspires.ftc.teamcode.utilities.LockState;
import org.firstinspires.ftc.teamcode.utilities.MoveSlidesState;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
@Config
public class Comp2Testing extends LinearOpMode {
    public static double START_X = 24, START_Y = 4.5, START_DEG = 270;
    private Pose2d START = new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    public static double MIDDLE_X = 36, MIDDLE_Y = 12, MIDDLE_DEG = 180;
    public static double HIGH_X = 17, HIGH_Y = 8.5, HIGH_DEG = -90;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        hardwareHandler.setDrive(drive);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        leftEncoder.reset();
        rightEncoder.reset();
        frontEncoder.reset();
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        drive.setPoseEstimateAndTrajEnd(START);
        Trajectory traj2 = drive.trajectoryBuilder(drive.getLastTrajEnd())
                .forward(5)
                .build();
        MoveWithRoadrunner move2 = new MoveWithRoadrunner("middleToHigh", traj2, drive);
        MoveSlidesState slides = new MoveSlidesState("slideToHigh", hardwareHandler, SlidePosition.FIVE_CONE);
        LockState lock = new LockState("middleToHigh + slideToHigh", new ArrayList<>(Arrays.asList(move2, slides)));
        lock.putNextState("next", new PlaceholderState());
        //move1.putNextState("next", move2);
        AbState currState = lock;
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
            hardwareHandler.updateSlides();
            for (TelemetryObj obj : currState.getTelemetries()) {
                telemetry.addData(obj.getCaption(), obj.getContent());
            }
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("front", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
