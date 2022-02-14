// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Indexer;

import org.opencv.photo.Photo;
import org.team639.lib.Constants;
import org.team639.lib.PhotoelectricSensor;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpitCargo extends CommandBase {
  /** Creates a new SpitCargo. */
  private Shooter shooter;
  private Indexer indexer;
  private Acquisition acquisition;


  /** Creates a new spitCargo. */
  public SpitCargo(Shooter shooter, Acquisition acquisition, Indexer indexer) {
    this.shooter = shooter;
    this.acquisition = acquisition;
    this.indexer = indexer;
    addRequirements(shooter);
    addRequirements(acquisition);
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexer.topDetected() || indexer.bottomDetected()){
      shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
      indexer.setIndexMotor(-1);
      acquisition.spinAcquisitionOut(1);
    }
    else{
      shooter.setSpeed(0);
      indexer.setIndexMotor(0);
      acquisition.stopAcquisitionMotor();
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
