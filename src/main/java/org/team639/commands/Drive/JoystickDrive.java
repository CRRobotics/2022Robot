// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.RobotContainer;
import org.team639.RobotContainer.DriveLayout;
import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Controls the robots movements
 */
public class JoystickDrive extends CommandBase {
  private DriveTrain driveTrain;

  /** Creates a new JoystickDrive. */
  public JoystickDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
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
    DriveLayout currMode = RobotContainer.getDriveLayout();
    switch(currMode)
    {
      case Arcade:
        arcadeDrive(ControllerWrapper.DriverController.getLeftY(), ControllerWrapper.DriverController.getRightX());

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  /**
   * Arcade drive given a speed and turning magnitude
   * @param speed Speed in percent
   * @param turnValue Magnitude of turning
   */
  public void arcadeDrive(double speed, double turnValue)
  {
    speed *= Constants.driveMultiplier;

    double turnMultiplier = 1 - speed;
    if (turnMultiplier < 1d / 3d) turnMultiplier = 1d / 3d;
    if (turnMultiplier > 2d / 3d) turnMultiplier = 2d / 3d;
    turnValue = turnValue * turnMultiplier;

    double left = speed + turnValue;
    double right = speed - turnValue;

    driveTrain.setSpeedsPercent(left, right);
  }

  public void cheezyDrive(double throttle, double wheel, boolean isQuickTurn)
  {

  }
  // HOW TO IMPLEMENT:
  //Three values: Throttle, Wheel and Quickturn
  //For turn in place, you can override the normal drive mode by checking if the throttle is pressed rather than if override button is pressed
  //Implement by copying cheesy poofs 2016 cheezy drive
}
