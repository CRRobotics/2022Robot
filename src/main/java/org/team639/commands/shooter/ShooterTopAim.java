// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.shooter;

import org.team639.controlboard.ControllerWrapper;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterTopAim extends CommandBase {

  private Shooter shooter;
  private double position = 0;
  /** Creates a new shooterAimUp. */
  public ShooterTopAim(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  
  /**Moves the shooter linear actuators up or down based on trigger input.
   * 
   */
  @Override
  public void execute() {
    if(ControllerWrapper.DriverController.getLeftTriggerAxis() > 0.1){
      position += 0.1;
    }
    else if(ControllerWrapper.DriverController.getRightTriggerAxis() > 0.1){
      position += -0.1;
    }
    shooter.actuatorMove(position);
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
