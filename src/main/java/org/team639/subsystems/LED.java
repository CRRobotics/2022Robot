// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.ctre.phoenix.led.CANdle;

import org.team639.Robot;
import org.team639.lib.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.Ports.CANdle.candleID);

  /**
   * Sets the CANdle LED based upon current orientation of the robot
   */
  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("Swapcade Mode", true))
      m_candle.setLEDs(0,255,0);
    else
      m_candle.setLEDs(255,0,0);


    // if(Math.abs(Robot.getAngleToTarget()) < 25)
    //   m_candle.setLEDs(0,255,0);
    // else
    //   m_candle.setLEDs(255,0,0);


  }
}
