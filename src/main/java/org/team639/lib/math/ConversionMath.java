// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib.math;

import org.team639.lib.Constants;
import org.team639.lib.states.GearMode;
import org.team639.subsystems.DriveTrain;


/**
 * Conversion of drive constants
 */
public class ConversionMath {
    /**
     * Returns the gear ratio based on current mode
     * @param mode Current gear mode
     * @return Ratio to be returned
     */
    public static double getGearRatio(GearMode mode)
    { 
        double ratio;
        if(mode == GearMode.high)
            ratio = Constants.DriveConstants.highGearRatio;
        else
            ratio = Constants.DriveConstants.lowGearRatio;
        return ratio;
    }


    /**
     * Converts encoder ticks of falcon 5 hundos to meters
     * @param ticks Ticks to be converted
     */
    public static double ticksToMeters(double ticks, double ratio)
    {   
        double rotation = (ticks / Constants.DriveConstants.ticksPerRevolution) / ratio; 

        return rotation * Constants.DriveConstants.wheelCircumference;
    }

    /**
     * Converts meters of falcon 5 hundos to ticks
     * @param meters meters to be converted
     */
    public static double metersToTicks(double meters, double ratio)
    {
        double rotations = meters / Constants.DriveConstants.wheelCircumference;
        return rotations * ratio * Constants.DriveConstants.ticksPerRevolution;
    }
}
