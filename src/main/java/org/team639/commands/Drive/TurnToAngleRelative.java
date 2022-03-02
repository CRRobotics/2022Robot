// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.Robot;
import org.team639.lib.Constants;
import org.team639.lib.states.GearMode;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToAngleRelative extends CommandBase {
  private DriveTrain driveTrain;
  private double setpoint;
  private boolean clockwise;

  private GearMode lastGear;
  private PIDController turnController = new PIDController(Constants.AutoConstants.autoRotateP, Constants.AutoConstants.autoRotateI, Constants.AutoConstants.autoRotateD);

  /** Creates a new TurnToAngleRelative. */
  public TurnToAngleRelative(DriveTrain driveTrain, double angle, boolean useVisions) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    if(useVisions && !Robot.lockedOn()) { end(true); }
    angle = useVisions ? Robot.getAngleToTarget() : angle % 360;

    this.clockwise = Math.signum(angle) > 0 ? true : false;
    setpoint = Math.abs(angle) + driveTrain.getHeading();

    turnController.setTolerance(Constants.AutoConstants.autoRotateThreshHold, Constants.AutoConstants.autoRotateThreshHoldVelo);
    turnController.setSetpoint(setpoint); 

    lastGear = driveTrain.getGearMode();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setSpeedsPercent(0, 0);
    if(lastGear.equals(GearMode.high))
      driveTrain.toggleGearLow();
    System.out.println("Auto rotate initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(clockwise)
      driveTrain.turnCommand(turnController.calculate(driveTrain.getHeading(), setpoint));
    else
      driveTrain.turnCommand(-turnController.calculate(driveTrain.getHeading(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeedsPercent(0, 0);
    if(lastGear.equals(GearMode.high))
      driveTrain.toggleGearHigh();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turnController.atSetpoint())
      return true;
    return false;
  }
}
