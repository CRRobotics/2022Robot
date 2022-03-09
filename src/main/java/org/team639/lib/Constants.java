// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.lib;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * All robot constants
 */
public interface Constants
{
    public interface Ports
    {
        public interface Drive
        {
            public static final int leftMainID = 10;
            public static final int leftFollowerID = 11;
            public static final int rightMainID = 13;
            public static final int rightFollowerID = 12;

            public static final int shifterID = 2;
        }

        public interface Indexer
        {
            public static final int indexMotorID = 7;
            public static final int indexBottomSensorID = 0;
            public static final int indexTopSensorID = 1;
        }

        public interface Shooter
        {
            public static final int mainID = 3;
            public static final int followID = 6;
            public static final int followActuatorID = 0;
            public static final int mainActuatorID = 1;

        }

        public interface Acquisition
        {

            public static final int acquisitionPortMain = 5;
            public static final int acquisitionPortFollow = 4;

            public static final int acquisitionRetract = 0;
            public static final int acquisitionExtend = 1;
        }

        public interface Climber
        {
            public static final int climbID = 3;
        }

        public interface PneumaticsModuleType
        {
            public static final int phCompressorID = 1;
        }

        public interface CANdle
        {
            public static final int candleID = 14;
        }

    }

    public interface AutoConstants
    {
        // public static final double kS = 0.52484;
        // public static final double kV = 6.1517;
        // public static final double kA = 0.22645;
        public static final double kS = .66379;
        public static final double kV = 4.0693;
        public static final double kA = 0.3922;

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

        
        // public static final double autoRotateP = .035;
        // public static final double autoRotateI = 0.0;
        // public static final double autoRotateD = 0.055;
        public static final double autoRotateP = 0.013783;
        public static final double autoRotateI = 0.0;
        public static final double autoRotateD = 0.0;

        public static final double autoForwardP = 0.00015;
        public static final double autoForwardI = 0.0;
        public static final double autoForwardD = 0.0;

        public static final double autoRotateThreshHold = 2.0;
        public static final double autoRotateThreshHoldVelo = 10.0;
        public static final double autoForwardThreshhold = 0.25;

        public interface pathNames
        {
            public static final String FenderToFender = "3BallFender";
        }

    }

    public interface DriveConstants
    {
       // public static final double chassisWidth = 0.74695;
       public static final double chassisWidth = 0.54695;
        public static final double wheelDiameter = .1016;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double driveMultiplier = 0.8; //tune this down a bit....maybe

        public static final double kDriveRampSeconds = 0.3;

        public static final double supplyCurrentLimiter = 30;
        public static final double supplyCurrentThreshHold = 35;

        public static final double lowGearRatio = 12.255;
        public static final double highGearRatio = 5.392;

        public static final double ticksPerRevolution = 2048;

        //Curvature Drive Constants
        public static final double kWheelDeadband = 0.1;
        public static final double kTurnSensitivity = 1;
        public static final double kThrottleDeadband = 0.02;
        public static final double overrideThreshhold = 0.1;
    }

    public interface ShooterConstants
    {
        public static final double reverseIndexSpeed = -0.2;
        public static final double reverseIndexWhenShootingTime = 150;
        public static final double spinUpTime = 1400;

        public static final double pureShootingTime = spinUpTime + reverseIndexWhenShootingTime + 1700;

        public static final int fenderRPM = 3000;
        public static final double fenderAngle = 0.9;

        public static final double shooterP = 0.0002;
        public static final double shooterI = 0.0;
        public static final double shooterD = 0.0002;
        public static final double shooterFF = 0.00018;

    }

    public interface IndexerConstants
    {
        public static final int indexSensorDistance = 10;
        public static final double indexMotorSpeed = 1;
        public static final double indexFeedSpeed = 0.8;
    }

    public interface AcquisitionConstants
    {
        public static final double acquisitionSpeed = 1;
        public static final double acquisitionSpeedSlow = 0.2;
    }

    public interface ControlboardConstants
    {
        public static final double kJoystickThreshold = 0.05;
        public static final double defaultCommandTimeout = 0.2;
    }

    public interface DJConstants
    {
        public static final String industry_baby = "industryBaby.chrp";
        public static final String old_town_road = "oldTownRoad.chrp";
    }
}
