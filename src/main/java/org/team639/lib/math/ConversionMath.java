// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib.math;

import org.team639.lib.Constants;
import org.team639.lib.GearMode;
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
        double ratio = Constants.highGearRatio;
        
        return ratio;
    }


    /**
     * Converts encoder ticks of falcon 5 hundos to meters
     * @param ticks Ticks to be converted
     */
    public static double ticksToMeters(double ticks)
    {   
        double rotation = (ticks / Constants.ticksPerRevolution) / getGearRatio(DriveTrain.currGear); 

        return rotation * Constants.wheelCircumference;
    }

    /**
     * Converts encoder ticks of falcon 5 hundos to meters
     * @param ticks Ticks to be converted
     */
    public static double metersToTicks(double meters)
    {
        double rotations = meters / Constants.wheelCircumference;
        return rotations * getGearRatio(DriveTrain.currGear)* Constants.ticksPerRevolution;
    }
}
