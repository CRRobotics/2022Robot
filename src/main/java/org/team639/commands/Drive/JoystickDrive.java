// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.RobotContainer;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase {
  private DriveTrain driveTrain;

  /** Creates a new JoystickDrive. */
  public JoystickDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = RobotContainer.driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("Joystick drive initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
