package com.team1678.frc2020.paths;

import com.team1678.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 150.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxCentripetalAccel = 110.0;
    private static final double kMaxVoltage = 9.0;


    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    
    // Test forward
    public static final Pose2d kTestStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestFarPose = new Pose2d(12.0, 0.0, Rotation2d.fromDegrees(0.0));

    // Test turn
    public static final Pose2d kTestTurnPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    public static final Pose2d kTestTurnPoseTurned = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    
    // Test path
    public static final Pose2d kTestPathStartPose = new Pose2d(140.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathFarPose = new Pose2d(155.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathTurnPose = new Pose2d(155.0, -140.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kTestPathTurnPoseTurned = new Pose2d(155.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathReturnPose = new Pose2d(140.0, -140.0, Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPathResetPose = new Pose2d(140.0, -140.0, Rotation2d.fromDegrees(180.0));
    public static final Pose2d kTestPathResetPoseTurned = new Pose2d(140.0, -140.0, Rotation2d.fromDegrees(0.0));

    public class TrajectorySet {
        // public class MirroredTrajectory {
        //     public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
        //         this.right = right;
        //         this.left = TrajectoryUtil.mirrorTimed(right);
        //     }

        //     public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
        //         return left ? this.left : this.right;
        //     }

        //     public final Trajectory<TimedState<Pose2dWithCurvature>> left;
        //     public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        // }

        public final Trajectory<TimedState<Pose2dWithCurvature>> testForwardPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTurnPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPathForward;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPathTurn;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPathReturn;

        private TrajectorySet() {
            testForwardPath = getTestForwardPath();
            testTurnPath = getTestTurnPath();
            testPathForward = getTestPathForward();
            testPathTurn = getTestPathTurn();
            testPathReturn = getTestPathReturn();        
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestForwardPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestStartPose);
            waypoints.add(kTestFarPose);
            return generateTrajectory(
                    false, 
                    waypoints, 
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                    kMaxVelocity, 
                    kMaxAccel, 
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTurnPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestTurnPose);
            waypoints.add(kTestTurnPoseTurned);
            return generateTrajectory(
                    false, 
                    waypoints, 
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
                    kMaxVelocity, 
                    kMaxAccel, 
                    kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPathForward() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestPathStartPose);
            waypoints.add(kTestPathFarPose);
            return generateTrajectory(
                false, 
                waypoints, 
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
                kMaxVelocity, 
                kMaxAccel, 
                kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPathTurn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestPathTurnPose);
            waypoints.add(kTestPathTurnPoseTurned);
            return generateTrajectory(
                false, 
                waypoints, 
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
                kMaxVelocity, 
                kMaxAccel, 
                kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPathReturn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestPathReturnPose);
            waypoints.add(kTestPathResetPose);
            waypoints.add(kTestPathResetPoseTurned);
            return generateTrajectory(
                false, 
                waypoints, 
                Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
                kMaxVelocity, 
                kMaxAccel, 
                kMaxVoltage);
        }
    }
}
