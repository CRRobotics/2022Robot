// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.lib.states.GearMode;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleGears extends CommandBase {
  private DriveTrain driveTrain;
  /** Creates a new ToggleGears. */
  public ToggleGears(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ToggleGears initialized");
    if(driveTrain.getGearMode().equals(GearMode.low))
      driveTrain.toggleGearHigh();
    else
      driveTrain.toggleGearLow();

  }

  public boolean isFinished()
  {
    return true;
  }

  public void end(boolean interrupted)
  {
    System.out.println("Gears Shifted");
  }

}
