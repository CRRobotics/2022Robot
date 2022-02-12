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
public interface Constants 
{
    public interface Ports
    {
        public interface Drive
        {
            public static final int leftMainID = 12;
            public static final int leftFollowerID = 13;
            public static final int rightMainID = 11;
            public static final int rightFollowerID = 10;
        }

        public interface Indexer
        {
            public static final int indexMotorID = 1;
        }

        public interface PneumaticsModuleType
        {
            public static final int shifterID = 0;
            public static final int phCompressorID = 1;
        }

        public interface Acquisition
        {
            public static final int acquisitionMotor1port = 1;
            public static final int acquisitionMotor2port = 2;
        }
    }

    public interface AutoConstants
    {
        public static final double kS = .132;
        public static final double kA = 2;
        public static final double kV = .0209;

        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxSpeedMetersPerSecond = 2;

        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.chassisWidth);
    
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS,
                kV,
                kA),
            kinematics,
            12);
        
        public static final TrajectoryConfig config = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(kinematics)
                .addConstraint(autoVoltageConstraint);

        
        public static final double autoRotateP = .035;
        public static final double autoRotateI = 0.0;
        public static final double autoRotateD = 0.055;

        public static final double autoForwardP = 0.00015;
        public static final double autoForwardI = 0.0;
        public static final double autoForwardD = 0.0;

        public interface pathNames
        {
            public static final String FenderToFender = "3BallFender";
        }

    }

    public interface DriveConstants
    {
        public static final double chassisWidth = 1.0;
        public static final double wheelDiameter = .1016;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double driveMultiplier = 1; //tune this down a bit....maybe

        public static final double kDriveRampSeconds = 0.3;
        
        public static final double supplyCurrentLimiter = 30;
        public static final double supplyCurrentThreshHold = 35;
           
        //TODO: Find this
        public static final double lowGearRatio = 18;
        public static final double highGearRatio = 18;

        public static final double ticksPerRevolution = 2048;

        //Curvature Drive Constants
        public static final double kWheelDeadband = 0.1;
        public static final double kTurnSensitivity = 1;
        public static final double kThrottleDeadband = 0.02;
        public static final double overrideThreshhold = 0.1;

        
    }

    public interface ControlboardConstants
    {
        //Control Board Constants
        public static final double kJoystickThreshold = 0.05;
    }

    //Indexer Constants
    public static final int indexMotorID = 0;
    public static final int indexSensorDistance = 10;
    public static final double indexMotorSpeed = 0.125;

    //Acquisition constants
    public static final double acquisitionMotorSpeed = 0.5;

}
