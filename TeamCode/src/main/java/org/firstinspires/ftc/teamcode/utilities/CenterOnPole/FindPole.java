package org.firstinspires.ftc.teamcode.utilities.CenterOnPole;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.EMPTY_DISTANCE;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_ACC;
import static org.firstinspires.ftc.teamcode.utilities.CenterOnPole.CenterOnPoleState.SLOW_VEL;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderForwardState;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.DriveConstants;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;

import java.util.HashMap;

@Config
public class FindPole extends AbState {
    private HardwareHandler hardwareHandler;
    private boolean backFirst;
    private AbState currState;
    public static double DISTANCE = 5;
    private SampleMecanumDrive drive;
    FindPole(String name, HardwareHandler hardwareHandler, boolean backFirst) {
        super(name, "next", "error");
        this.hardwareHandler = hardwareHandler;
        this.backFirst = backFirst;
        drive = hardwareHandler.getDrive();
    }

    @Override
    public void init() {
        Trajectory traj;
        if (backFirst) {
            traj = drive.trajectoryBuilder(drive.getLastTrajEnd())
                    .splineToConstantHeading(new Vector2d(-DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(2 * DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(-DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .build();
        }
        else {
            traj = drive.trajectoryBuilder(drive.getLastTrajEnd())
                    .splineToConstantHeading(new Vector2d(DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(-2 * DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .splineToConstantHeading(new Vector2d(DISTANCE, 0), Math.toRadians(0), SLOW_VEL, SLOW_ACC)
                    .build();
        }
        MoveWithRoadrunner move = new MoveWithRoadrunner("backAndForth", traj, drive);
        move.putNextState("next", new PlaceholderState());
        currState = move;
        currState.init();
        setSubStates(currState);
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (hardwareHandler.getLeftPoleSensor(INCH) < EMPTY_DISTANCE) {
            hardwareHandler.setDriveTrainPowers(0, 0, 0, 0);
            return getNextState("next");
        }
        if (currState instanceof PlaceholderState) return getNextState("error");
        return this;
    }

    @Override
    public void run() {
        currState.run();
        AbState nextState = currState.next();
        if (!nextState.equals(currState)) {
            setSubStates(nextState);
        }
        currState = nextState;
    }
}
