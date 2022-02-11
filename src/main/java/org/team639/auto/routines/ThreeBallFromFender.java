// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.auto.routines;

import org.team639.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeBallFromFender extends CommandBase {

  private DriveTrain driveTrain;
  /** Creates a new ThreeBallFromFender. */
  public ThreeBallFromFender(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  public void initialize(){
    new SequentialCommandGroup(driveTrain.FenderToFender
    );
    System.out.println("3 Ball Fender Initializing");
  }

  public void end(boolean interrupted){
    System.out.println("3BallFender outta here");
  }

  public String getName()
  {
    return "3 Ball from Fender";
  }

  public Command getCommand() {
    // TODO Auto-generated method stub
    return driveTrain.FenderToFender;
  }
}
