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

    //TODO: Figure out what these are
    public static final int leftMainID = 0;
    public static final int leftFollowerID = 1;
    public static final int rightMainID = 2;
    public static final int rightFollowerID = 3;

    public static final double driveMultiplier = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kDriveRampSeconds = 0.3;

    public static final double supplyCurrentLimiter = 30;
    public static final double statorCurrentLimiter = 30;
    public static final double supplyCurrentThreshHold = 35;
    public static final double statorCurrentThreshHold = 35; 

    //TODO: Find this
    public static final double driveTrainGearRatio = 18;
    public static final double ticksPerRevolution = 2048;

    public static final double autoRotateP = .0001;
    public static final double autoRotateI = 0.0;
    public static final double autoRotateD = 0.0;

    //Curvature Drive Constants
    public static final double kWheelDeadband = 0.1;
    public static final double kTurnSensitivity = 1;
    public static final double kThrottleDeadband = 0.02;
    public static final double overrideThreshhold = 0.1;
    



    //Control Board Constants
    public static final double kJoystickThreshold = 0.2;

}
