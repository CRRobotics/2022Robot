// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngle extends CommandBase {
  private DriveTrain driveTrain;
  private double setpoint;
  private boolean clockwise;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    angle %= 360;

    this.clockwise = Math.signum(angle) > 0 ? true : false;
    setpoint = Math.abs(angle) + driveTrain.getHeading();

    driveTrain.turnController.setTolerance(2);
    driveTrain.turnController.setTolerance(setpoint, .003);
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
    double currMultiplier = driveTrain.turnController.calculate(driveTrain.getHeading(), setpoint);
    if(clockwise)
    {
      driveTrain.setSpeedsPercent(1 * currMultiplier, -1 * currMultiplier);
    }
    else
    {
      driveTrain.setSpeedsPercent(-currMultiplier, 1 * currMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeedsPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(driveTrain.turnController.atSetpoint())
      return true;
    return false;
  }
}
