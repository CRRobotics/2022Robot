// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveForward extends CommandBase {
  private DriveTrain driveTrain;

  private PIDController leftController = new PIDController(Constants.AutoConstants.autoForwardP, Constants.AutoConstants.autoForwardI, Constants.AutoConstants.autoForwardD);
  private PIDController rightController = new PIDController(Constants.AutoConstants.autoForwardP, Constants.AutoConstants.autoForwardI, Constants.AutoConstants.autoForwardD);

  private double startLeft;
  private double startRight;
  private double targetLeft;
  private double targetRight;

  private double errorRight;
  private double errorLeft;

  /** Creates a new AutoDriveForward. */
  public AutoDriveForward(DriveTrain driveTrain, double distance) {
    leftController.setTolerance(Constants.AutoConstants.autoForwardThreshhold);
    rightController.setTolerance(Constants.AutoConstants.autoForwardThreshhold);

    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    distance = Math.abs(distance);

    startLeft = driveTrain.getLeftPostion(); startRight = driveTrain.getRightPostion();
    targetLeft = startLeft + distance;
    targetRight = startRight + distance;

    errorLeft = targetLeft - startLeft;
    errorRight = targetRight - startRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setSpeedsPercent(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    errorLeft = targetLeft - driveTrain.getLeftPostion();
    errorRight = targetRight - driveTrain.getRightPostion();

    double leftMultiplier = leftController.calculate(errorLeft);
    double rightMultiplier = rightController.calculate(errorRight);

    driveTrain.setSpeedsPercent(1 * leftMultiplier, 1 * rightMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeedsPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(leftController.atSetpoint() && rightController.atSetpoint())
      return true;
    return false;
  }

}
