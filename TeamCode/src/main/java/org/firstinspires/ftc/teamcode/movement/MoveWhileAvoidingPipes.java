package org.firstinspires.ftc.teamcode.movement;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.movement.roadrunner.MoveWithRoadrunner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Queue;

public class MoveWhileAvoidingPipes extends AbState {
    private Pose2d startingPose, endingPose;
    private SampleMecanumDrive drive;
    private MoveWithRoadrunner move;
    public MoveWhileAvoidingPipes(String name, Pose2d startingPose, Pose2d endingPose, SampleMecanumDrive drive) {
        super(name, "next");
        this.startingPose = startingPose;
        this.endingPose = endingPose;
        this.drive = drive;
    }

    @Override
    public void init() {
        // convert global coordinates to grid coordinates
        int gsx = (int) Math.floor(startingPose.getX()/24.0), gsy = (int) Math.floor(startingPose.getY()/24.0),
                gex = (int) Math.floor(endingPose.getX()/24.0), gey = (int) Math.floor(endingPose.getY()/24.0);

        /*
        APPROACH:
        - change coordinates into grid coordinates
        - breadth first search to ending grid position
        - backtrack to find the shortest path to ending grid position
        - create trajectory to do that
         */
        // initialize variables for breadth first search
        PriorityQueue<int[]> next = new PriorityQueue<>();
        next.add(new int[]{gsx, gsy});
        int[][] visited = new int[6][6];
        int[][] dist = new int[6][6];
        dist[gsx][gsy] = 0;

        // implement BFS
        while (true) {
            int[] curr = next.poll();
            int cx = curr[0], cy = curr[1], cdist = dist[cx][cy];

            // if we're at the goal, stop
            if (cx == gex && cy == gey) {
                break;
            }
            // if we're revisiting grid space, don't
            if (visited[cx][cy] == 1) {
                continue;
            }

            // set that we've visited this square + add next grid squares to queue
            visited[cx][cy] = 1;
            ArrayList<int[]> adj = adjacent(cx, cy);
            for (int[] coord : adj) {
                int tx = coord[0], ty = coord[1];
                // track the distance from start
                dist[tx][ty] = cdist + 1;
            }
            next.addAll(adj);
        }

        // backtrack to find shortest path
        // init vars
        int leastDist = dist[gex][gey];
        ArrayList<int[]> path = new ArrayList<>();
        path.add(new int[]{gex, gey});
        /*
        ALGORITHM:
        - start at end point
        - the path will include the grid space that's adjacent and has a 1 lesser distance from start
         */
        for (int i = 0; i < leastDist; i++) {
            int[] currPath = path.get(i);
            int cx = currPath[0], cy = currPath[1];
            int cdist = dist[cx][cy];
            ArrayList<int[]> adj = adjacent(cx, cy);
            // go through every adjacent coordinate, if it has a one lesser distance, accept into path
            for (int[] coord : adj) {
                int tx = coord[0], ty = coord[1];
                if (dist[tx][ty] + 1 == cdist) {
                    path.add(coord);
                    break;
                }
            }
        }
        Collections.reverse(path);

        // trajectory stuff
        TrajectoryBuilder trajBuilder = drive.trajectoryBuilder(startingPose);
        double currAngle = startingPose.getHeading(), endAngle = endingPose.getHeading(),
                dAngle = (endAngle - currAngle) / path.size();
        boolean first = true;
        int[] prevDir = new int[3];
        int[] prevCoord = new int[3];
        for (int[] coord : path) {
            int[] dir = new int[]{coord[0] - prevCoord[0], coord[1] - prevCoord[1], coord[2] - prevCoord[2]};
            if (first || !Arrays.equals(dir, prevDir)) {
                currAngle += dAngle;
                double[] globalCoord = gridToGlobal(coord);
                Pose2d targetPose = new Pose2d(globalCoord[0], globalCoord[1], currAngle);
                trajBuilder.lineToLinearHeading(targetPose);
            }
            prevCoord = coord;
            prevDir = dir;
            first = false;
        }
        trajBuilder.lineToLinearHeading(endingPose);
        Trajectory traj = trajBuilder.build();
        
        move = new MoveWithRoadrunner(super.name, traj, drive);
        move.putNextState("next", getNextState("next"));
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        return move;
    }

    @Override
    public void run() {

    }

    public ArrayList<int[]> adjacent(int i, int j) {
        ArrayList<int[]> adj = new ArrayList<>();
        if (i != 6 && j != 6) {
            adj.add(new int[]{i+1,j+1});
        }
        if (i != 6 && j != 0) {
            adj.add(new int[]{i+1,j-1});
        }
        if (i != 0 && j != 6) {
            adj.add(new int[]{i-1,j+1});
        }
        if (i != 6 && j != 0) {
            adj.add(new int[]{i-1,j-1});
        }
        return adj;
    }

    public double[] gridToGlobal(int[] coord) {
        int i = coord[0], j = coord[1];
        return new double[]{24*i + 12, 24*j + 12};
    }
}
