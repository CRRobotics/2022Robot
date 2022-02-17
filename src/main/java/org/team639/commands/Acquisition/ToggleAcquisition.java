// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Acquisition;

import org.team639.subsystems.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleAcquisition extends CommandBase {
  private Acquisition acquisition;
  /** Creates a new ToggleAcquisition. */
  public ToggleAcquisition(Acquisition acquisition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.acquisition = acquisition;
    addRequirements(acquisition);
  }

  private void addRequirements(Acquisition acquistion2) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(acquisition.isAcquisitionDown())
      acquisition.acquisitionUp();
    else
      acquisition.acquisitionDown();
  }

  
}