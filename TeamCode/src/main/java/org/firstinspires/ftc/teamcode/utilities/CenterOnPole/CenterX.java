package org.firstinspires.ftc.teamcode.utilities.CenterOnPole;

import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.EMPTY_DISTANCE;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SEARCH_DISTANCE;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_ACC;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_VEL;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.TOLERANCE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.structures.PIDFController;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

import java.util.HashMap;

@Config
public class CenterX extends AbState {
    private HardwareHandler hardwareHandler;
    private SampleMecanumDrive drive;
    private double leftDistance, rightDistance, target;
    private PIDFController pid;
    public static double P = 0.04, I = 0, D = 0, S = 0.125;
    private boolean noCones;
    private Pose2d startPose;
    CenterX(String name, HardwareHandler hardwareHandler, boolean noCones) {
        super(name, "next");
        this.hardwareHandler = hardwareHandler;
        this.drive = hardwareHandler.getDrive();
        this.noCones = noCones;
        startPose = drive.getLastTrajEnd();
    }

    @Override
    public void init() {
        pid = new PIDFController(P, I, D, S, 0, 0);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (leftDistance > EMPTY_DISTANCE || rightDistance > EMPTY_DISTANCE) {
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setLastTrajEnd(drive.getPoseEstimate());

            boolean back = drive.getPoseEstimate().getX() > startPose.getX();
            CenterY recenter = new CenterY("recenter", hardwareHandler, 1, noCones, back);
            CenterX centerX = new CenterX("centerX", hardwareHandler, noCones);
            recenter.putNextState("next", centerX);
            centerX.putNextState("next", getNextState("next"));
            return recenter;

            /*
            Trajectory traj;
            Pose2d last = drive.getLastTrajEnd();
            traj = drive.trajectoryBuilder(last)
                    .lineToConstantHeading(new Vector2d(-1, 0), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(1, 0), Math.toRadians(last.getHeading()), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(-last.getHeading()), SLOW_VEL, SLOW_ACC)
                    .build();
            //unstuck
            MoveWithRoadrunner move = new MoveWithRoadrunner("backAndForth", traj, drive);
            //stuck
            move.putNextState("next", new PlaceholderState());
            return move;

             */
        }

        if (Math.abs(leftDistance - rightDistance) < TOLERANCE) {
            drive.setMotorPowers(0, 0, 0, 0);
            drive.setLastTrajEnd(drive.getPoseEstimate());
            return getNextState("next");
        }
        return this;
    }

    @Override
    public void run() {
        leftDistance = hardwareHandler.getLeftPoleSensor(DistanceUnit.INCH);
        if (leftDistance < EMPTY_DISTANCE) {
            rightDistance = hardwareHandler.getRightPoleSensor(DistanceUnit.INCH);
            target = rightDistance - leftDistance;
            double input = Math.min(Math.max(pid.getInput(target), -0.3), 0.3);
            drive.setDrivePower(new Pose2d(0, input, 0));
        }
        drive.update();
    }
}
