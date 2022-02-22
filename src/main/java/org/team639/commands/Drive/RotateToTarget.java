// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.Robot;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RotateToTarget extends SequentialCommandGroup {
  /** Creates a new RotateToTarget. */
  public RotateToTarget(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new Autorotate(driveTrain, Robot.getAngleToTarget()));
  }

  public void initialize()
  {
    System.out.println("RotateToTarget initializing");
  }

  public void end(boolean interrupted)
  {
    System.out.println("RotateToTarget dippin'");
  }
}
