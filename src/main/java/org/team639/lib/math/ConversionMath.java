// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib.math;

import org.team639.lib.Constants;

/**
 * Conversion of drive constants
 */
public class ConversionMath {
    /**
     * Converts encoder ticks of falcon 5 hundos to meters
     * @param ticks Ticks to be converted
     */
    public static double ticksToMeters(double ticks)
    {
        double rotation = (ticks / Constants.ticksPerRevolution) / Constants.driveTrainGearRatio; 
        return rotation * Constants.wheelCircumference;
    }

    /**
     * Converts encoder ticks of falcon 5 hundos to meters
     * @param ticks Ticks to be converted
     */
    public static double metersToTicks(double meters)
    {
        double rotations = meters / Constants.wheelCircumference;
        return rotations * Constants.driveTrainGearRatio * Constants.ticksPerRevolution;
    }
}
