// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Indexer;

import org.team639.lib.Constants;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualIndexer extends CommandBase {
  private Indexer indexer;
  private Shooter shooter;

  /** Creates a new ManualIndexer. */
  public ManualIndexer(Indexer indexer, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(indexer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
    indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    indexer.setIndexMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
