// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.controlboard;

import org.team639.lib.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class TriggerButton extends Button 
{
    XboxController joystick;
    int handID;

    /**
     * Trigger button boolean
     * @param joystick Joystick to use
     * @param handID Hand to use. 0 for left, 1 for right
     */
    public TriggerButton(XboxController joystick, int handID)
    {
        this.joystick = joystick;
        this.handID = handID;
    }

    @Override
    public boolean get()
    {
        return handID == 0 ? handleDeadband(joystick.getLeftTriggerAxis(), Constants.DriveConstants.kThrottleDeadband) > 0 : handleDeadband(joystick.getRightTriggerAxis(), Constants.DriveConstants.kThrottleDeadband) > 0;        
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
      }
}
