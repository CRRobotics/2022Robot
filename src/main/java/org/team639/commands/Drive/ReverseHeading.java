// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseHeading extends CommandBase {
  private DriveTrain driveTrain;
  public ReverseHeading(DriveTrain driveTrain)
  {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setHeading(!driveTrain.isReversedHeading());
  }
}
