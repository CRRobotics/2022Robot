// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;

import org.team639.Robot;
import org.team639.RobotContainer;
import org.team639.lib.Constants;
import org.team639.lib.states.LEDMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.Ports.CANdle.candleID);
  private LEDMode curr_led = RobotContainer.getLedMode();
  private int LedCount = 8;

  private final FireAnimation fire = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
  private final RgbFadeAnimation gamer_mode = new RgbFadeAnimation(0.7, 1, LedCount);

  /**
   * Sets the CANdle LED based upon current orientation of the robot
   */
  @Override
  public void periodic() {
    updateLED();


  }

  public void updateLED()
  {
     curr_led = RobotContainer.getLedMode();
     switch(curr_led)
     {
        case aimbot:
          aimLockmode();
          break;
        case swapcade:
          swapcadeMode();
          break;
        case fire:
          fireAnim();
          break;
        case gamerMode:
          rgbFade();
          break;
     }
  }

  public void swapcadeMode()
  {
    if(SmartDashboard.getBoolean("Swapcade Mode", true))
      m_candle.setLEDs(0,255,0);
    else
      m_candle.setLEDs(255,0,0);
  }

  public void aimLockmode()
  {
    if(Robot.lockedOn())
      m_candle.setLEDs(0,255,0);
    else
      m_candle.setLEDs(255,0,0);
  }

  public void fireAnim()
  {
    m_candle.animate(fire);
  }

  public void rgbFade()
  {
    m_candle.animate(gamer_mode);
  }
}
