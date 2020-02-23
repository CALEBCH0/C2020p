/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1678.frc2020.auto.actions;

import com.team1678.frc2020.subsystems.Drive;
import com.team254.lib.geometry.Rotation2d;

public class TurnToHeadingAction implements Action {
    private ReflectiveOperationException mTargetHeading;
    private Drive mDrive = Drive.getInstance();

    public TurnToHeadingAction(Rotation2d heading) {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished(){
        return mDrive.isDonewWithTurn();
    }

    @Override
    public void update() {
        // nothing done here, controller updates in
    }

    @Override
    public void done() {}

    @Override
    public void start() {
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}
