package com.team1678.frc2020.auto.modes;

import com.team1678.frc2020.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Rotation2d;
import com.team1678.frc2020.auto.AutoModeEndedException;
import com.team1678.frc2020.auto.actions.*;

public class TestPathMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectoryAction mTestPathForward;
    private DriveTrajectoryAction mTestPathTurn;
    private DriveTrajectoryAction mTestPathReturn;

    public TestPathMode() {
        mTestPathForward = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPathForward, true);
        mTestPathTurn = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPathForward, false);
        mTestPathTurn = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPathTurn, false);
        mTestPathReturn = new DriveTrajectoryAction(mTrajectoryGenerator.getTrajectorySet().testPathReturn, false); // resetposes might not be correct
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Test Path Auto");
        runAction(mTestPathForward);
        runAction(mTestPathTurn);
        runAction(new WaitAction(0.5));
        runAction(mTestPathReturn);
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
    }
}
