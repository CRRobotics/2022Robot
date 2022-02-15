// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Autorotate extends CommandBase {

  private PIDController turnController = new PIDController(Constants.AutoConstants.autoRotateP, Constants.AutoConstants.autoRotateI,
      Constants.AutoConstants.autoRotateD);
      
  private DriveTrain driveTrain;

  private double target;
  private double error;
  private boolean clockwise;

  /** Creates a new AutoRotate. */
  public Autorotate(DriveTrain driveTrain, double angle) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    angle %= 360;

    this.clockwise = Math.signum(angle) > 0 ? true : false;
    target = Math.abs(angle) + driveTrain.getHeading();
    
    //target = (Math.signum(angle) > 0) ? Math.abs(angle) + driveTrain.getHeading().getDegrees() : driveTrain.getHeading().getDegrees() - Math.abs(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setSpeedsPercent(0, 0);
    System.out.println("Auto rotate initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(clockwise)
    {
      error = target - driveTrain.getHeading();
      double currMultiplier = turnController.calculate(error);
      driveTrain.setSpeedsPercent(-1 * currMultiplier, 1 * currMultiplier);
    }
    else
    {
      error = Math.abs(target) - Math.abs(driveTrain.getHeading());
      double currMultiplier = turnController.calculate(error);
      driveTrain.setSpeedsPercent(currMultiplier, -1 * currMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeedsPercent(0, 0);
    System.out.println("Auto rotate headed out");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (error < 5)
      return true;
    return false;
  }
}
