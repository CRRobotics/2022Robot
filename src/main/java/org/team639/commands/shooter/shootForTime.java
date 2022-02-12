// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import org.team639.lib.Constants;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootForTime extends CommandBase {

  private Shooter shooter;
  private long startTime;
  private long shootTime;
  
  /** Creates a new shootForTime. */
  public ShootForTime(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    shootTime = Constants.shootTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() - startTime < shootTime){
      shooter.maintainLeftRPM(Constants.TargetRPM);
      shooter.maintainRightRPM(Constants.TargetRPM);
    }
    else{
      shooter.toggleleftOn(0);
      shooter.togglerightOn(0);
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
}
