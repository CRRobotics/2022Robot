// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

public class ShootClosedLoop extends CommandBase {
  private Indexer indexer;
  private Shooter shooter;
  private Acquisition acquisition;

  private long startTime;

  public ShootClosedLoop(Indexer indexer, Shooter shooter, Acquisition acquisition) {
      this.indexer = indexer;
      this.shooter = shooter;
      this.acquisition = acquisition;
      addRequirements(indexer, shooter, acquisition);
  }

  @Override
  public void initialize() {
      startTime = System.currentTimeMillis();
      shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
      indexer.setIndexMotor(-Constants.IndexerConstants.indexMotorSpeed);

      acquisition.acquisitionNeutral();
      shooter.setCoast();
  }

  @Override
  public void execute() {
      if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime)
      {
        indexer.setIndexMotor(0);
        shooter.setSpeedRPM(shooter.getSelectedRPM());
      }
     if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime + Constants.ShooterConstants.spinUpTime)
      {
        shooter.setSpeedRPM(shooter.getSelectedRPM());
        indexer.setIndexMotor(Constants.IndexerConstants.indexFeedSpeed);
        acquisition.spinAcquisition(acquisition.getAcquisitionSpeed());
      }   
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    indexer.setIndexMotor(0);
    acquisition.stopAcquisitionMotor();
    shooter.setBrake();
  }

  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() > startTime + Constants.ShooterConstants.pureShootingTime)
      return true;
    return false;
  }
}