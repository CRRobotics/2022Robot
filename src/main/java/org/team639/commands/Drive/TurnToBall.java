// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.Robot;
import org.team639.RobotContainer;
import org.team639.lib.Constants;
import org.team639.lib.states.AllianceColor;
import org.team639.lib.states.GearMode;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToBall extends CommandBase {
  private DriveTrain driveTrain;
  private double setpoint;

  private double angle;
  private boolean clockwise;

  private GearMode lastGear;
  private PIDController controller = new PIDController(Constants.AutoConstants.autoRotateP,
      Constants.AutoConstants.autoRotateI, Constants.AutoConstants.autoRotateD);

  /**
   * Creates relative turn to ball that uses vision tracking
   * @param driveTrain DriveTrain to use
   */
  public TurnToBall(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastGear = driveTrain.getGearMode();
    angle = RobotContainer.getAllianceColor().equals(AllianceColor.blue) ? Robot.getAngleToBallBlue() % 360.0 : Robot.getAngleToBallRed() % 360.0;
    clockwise = Math.signum(angle) > 0 ? true : false;
    //setpoint = clockwise ? Math.abs(angle) + driveTrain.getHeading() : driveTrain.getHeading() - Math.abs(angle);
    setpoint = driveTrain.getHeading() + angle;

    controller.setTolerance(Constants.AutoConstants.autoRotateThreshHold,
        Constants.AutoConstants.autoRotateThreshHoldVelo);
    controller.setSetpoint(setpoint);

    driveTrain.setSpeedsPercent(0, 0);
    if (lastGear.equals(GearMode.high))
      driveTrain.toggleGearLow();
    System.out.println("Auto rotate initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (clockwise)
      driveTrain.turnCommandR(controller.calculate(driveTrain.getHeading(), setpoint));
    else
      driveTrain.turnCommandL(-controller.calculate(driveTrain.getHeading(), setpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeedsPercent(0, 0);
    if (lastGear.equals(GearMode.high))
      driveTrain.toggleGearHigh();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (controller.atSetpoint())
      return true;
    return false;
  }
}
