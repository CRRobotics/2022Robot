// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

public class ShootClosedLoop extends CommandBase {
  private Indexer indexer;
  private Shooter shooter;
  private int rpm;

  private long startTime;

  public ShootClosedLoop(Indexer indexer, Shooter shooter, int rpm) {
      this.indexer = indexer;
      this.shooter = shooter;
      this.rpm = rpm;
      addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {
      startTime = System.currentTimeMillis();
      shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
      indexer.setIndexMotor(-Constants.IndexerConstants.indexMotorSpeed);
  }

  @Override
  public void execute() {
      if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime)
      {
        shooter.setSpeedRPM(rpm);
      }
      if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime + Constants.ShooterConstants.spinUpTime)
      {
        shooter.setSpeedRPM(rpm);
        indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
      }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() > startTime + Constants.ShooterConstants.pureShootingTime)
      return true;
    return false;
  }
}
