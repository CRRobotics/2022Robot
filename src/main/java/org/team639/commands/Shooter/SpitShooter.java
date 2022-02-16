// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpitShooter extends CommandBase {
  private Shooter shooter;
  private double speed;

  /** Creates a new SpitShooter. */
  public SpitShooter(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
