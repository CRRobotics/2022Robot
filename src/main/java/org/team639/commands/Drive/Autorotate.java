// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.lib.Constants;
import org.team639.lib.states.GearMode;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Autorotate extends PIDCommand {
  private DriveTrain driveTrain;
  private GearMode lastGear;

  /** Creates a new Autorotate. */
  public Autorotate(DriveTrain driveTrain, double targetAngle, boolean robotRelative) {
    super(driveTrain.turnController,
          driveTrain::getHeading,
          robotRelative ? (driveTrain.getHeading() + targetAngle) % 360 : targetAngle,
          output -> driveTrain.turnCommand(output),
          driveTrain
    );
    getController().enableContinuousInput(-180, 180);
    getController()
        .setTolerance(Constants.AutoConstants.autoRotateThreshHold, Constants.AutoConstants.autoRotateThreshHoldVelo);
    this.driveTrain = driveTrain;
    lastGear = driveTrain.getGearMode();
  }

  @Override
  public void initialize() {
    driveTrain.toggleGearLow();
  }

  @Override
  public void end(boolean interrupted) {
    if(lastGear.equals(GearMode.high))
      driveTrain.toggleGearHigh();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
