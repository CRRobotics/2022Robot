// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import org.team639.lib.Constants;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  
  private boolean climbExtended = false;
  Solenoid main = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.Climber.climbID);
  
  /** Creates a new Climber. */
  public Climber() {
    climberRetract();
  }

  public void climberExtend()
  {
    if(!climbExtended)
    {
      main.set(true);
      climbExtended = true;
    }
  }

  public void climberRetract()
  {
    if(climbExtended)
    {
      main.set(false);
      climbExtended = false;
    }
  }

  public boolean getClimberPosition()
  {
    return climbExtended;
  }
}
