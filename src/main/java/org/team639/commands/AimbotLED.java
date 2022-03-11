// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands;

import org.team639.subsystems.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimbotLED extends CommandBase {
  private LED led;
  /** Creates a new AimbotLED. */
  public AimbotLED(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.aimLockmode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
