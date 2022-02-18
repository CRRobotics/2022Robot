// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.Robot;
import org.team639.commands.Shooter.ShootOpenLoop;
import org.team639.subsystems.DriveTrain;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AimbotShot extends SequentialCommandGroup {
  /** Creates a new AimbotShot. */
  public AimbotShot(DriveTrain driveTrain, Indexer indexer, Shooter shooter) {
    addCommands(
      new Autorotate(driveTrain, Robot.getAngleToTarget()),
      new ShootOpenLoop(indexer, shooter)
    );
  }
}
