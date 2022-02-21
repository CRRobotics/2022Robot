// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Indexer;


import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpitCargo extends CommandBase {
  private Shooter shooter;
  private Indexer indexer;
  private Acquisition acquisition;
  
  /** Creates a new SpitCargo. */
  public SpitCargo(Shooter shooter, Indexer indexer, Acquisition acquisition) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.acquisition = acquisition;
    addRequirements(shooter, indexer, acquisition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acquisition.acquisitionNeutral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
    indexer.setIndexMotor(-Constants.IndexerConstants.indexMotorSpeed);
    acquisition.spinAcquisition(-acquisition.getAcquisitionSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    indexer.setIndexMotor(0);
    acquisition.stopAcquisitionMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
