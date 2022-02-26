// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpitShooter extends CommandBase {
  private Shooter shooter;
  private Indexer index;
  private Acquisition acquisition;
  private double speed;

  /** Creates a new SpitShooter. */
  public SpitShooter(Shooter shooter, Indexer index, Acquisition acquisition, double speed) {
    this.shooter = shooter;
    this.index = index;
    this.acquisition = acquisition;
    this.speed = speed;
    addRequirements(shooter, index, acquisition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeedRPM(shooter.getSelectedRPM());
    shooter.setCoast();
    acquisition.spinAcquisition(Constants.AcquisitionConstants.acquisitionSpeedSlow);
    index.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    acquisition.stopAcquisitionMotor();
    index.setIndexMotor(0);
    shooter.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
