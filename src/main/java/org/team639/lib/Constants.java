// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    //Drive Constants
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;
    
    public static final double chassisWidth = 0;
    public static final double wheelDiameter = .1524;
    public static final double wheelCircumference = 0.4788;

    //TODO: Figure out what these are
    public static final int leftMainID = 0;
    public static final int leftFollowerID = 1;
    public static final int rightMainID = 2;
    public static final int rightFollowerID = 3;

    //TODO: THESE ARE PLACEHOLDERS THAT WILL BREAK EVERYTHING(not really tho)
    public static final int shifterID = 0;
    public static final int phCompressorID = 1;
    public static final int maxCompressor = 110;

    public static final double driveMultiplier = 1; //tune this down a bit....maybe
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kDriveRampSeconds = 0.3;

    public static final double supplyCurrentLimiter = 30;
    public static final double supplyCurrentThreshHold = 35;

    //TODO: Find this
    public static final double driveTrainGearRatio = 18;
    public static final double ticksPerRevolution = 2048;

    public static final double autoRotateP = .00015;
    public static final double autoRotateI = 0.0;
    public static final double autoRotateD = 0.0;

    public static final double autoForwardP = 0.00015;
    public static final double autoForwardI = 0.0;
    public static final double autoForwardD = 0.0;


    //Curvature Drive Constants
    public static final double kWheelDeadband = 0.1;
    public static final double kTurnSensitivity = 1;
    public static final double kThrottleDeadband = 0.02;
    public static final double overrideThreshhold = 0.1;
    



    //Control Board Constants
    public static final double kJoystickThreshold = 0.2;

}
