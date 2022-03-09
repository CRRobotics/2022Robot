// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Climber;

import org.team639.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleClimber extends CommandBase {
  private Climber climb;

  /** Creates a new ToggleClimber. */
  public ToggleClimber(Climber climb) {
    this.climb = climb;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(climb.getClimberPosition())
      climb.climberRetract();
    else
      climb.climberExtend();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
