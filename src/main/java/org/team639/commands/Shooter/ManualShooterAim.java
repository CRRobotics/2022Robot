// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import org.team639.controlboard.ControllerWrapper;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualShooterAim extends CommandBase {

  private Shooter shooter;

  /** Creates a new ManualShooterAim */
  public ManualShooterAim(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(ControllerWrapper.DriverController.getLeftY() > 0.01){
    //   position += 0.1;
    // }
    // else if(ControllerWrapper.DriverController.getRightY() > 0.01){
    //   position += -0.1;
    // }

    shooter.setActuator(ControllerWrapper.DriverController.getLeftTriggerAxis());
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