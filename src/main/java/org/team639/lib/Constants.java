// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

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
    public static final double kS = .5;
    public static final double kA = .5;
    public static final double kV = .5;
    
    public static final double chassisWidth = 1;
    public static final double wheelDiameter = .1016;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.chassisWidth);

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.kS,
            Constants.kV,
            Constants.kA),
        kinematics,
        12);
    
    public static final TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kinematics)
            .addConstraint(autoVoltageConstraint);
    

    //TODO: Figure out what these are
    public static final int leftMainID = 12;
    public static final int leftFollowerID = 13;
    public static final int rightMainID = 11;
    public static final int rightFollowerID = 10;

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
    public static final double lowGearRatio = 18;
    public static final double highGearRatio = 18;

    public static final double ticksPerRevolution = 2048;

    public static final double autoRotateP = .035;
    public static final double autoRotateI = 0.0;
    public static final double autoRotateD = 0.055;

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
