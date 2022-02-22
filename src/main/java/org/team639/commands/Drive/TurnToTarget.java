// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.Robot;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToTarget extends CommandBase {
  private DriveTrain driveTrain;
  /** Creates a new TurnToTarget. */
  public TurnToTarget(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    new Autorotate(driveTrain, Robot.getAngleToTarget());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
