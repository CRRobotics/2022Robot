// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.team639.lib.Constants;
import org.team639.lib.math.ConversionMath;
import org.team639.lib.states.GearMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final Field2d simField = new Field2d();

    // Gyroscope and initial pose
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    private Pose2d startPosition = new Pose2d(new Translation2d(0, 0), gyro.getRotation2d());

    // Differential Control Systems
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    // Independent left and right PID controllers
    public PIDController leftPIDController = new PIDController(0.00012078, 0, 0);
    public PIDController rightPIDController = new PIDController(0.00012078, 0, 0);
    // public PIDController turnController = new
    // PIDController(Constants.AutoConstants.autoRotateP,
    // Constants.AutoConstants.autoRotateI,Constants.AutoConstants.autoRotateD);

    // Talon motor controllers
    public WPI_TalonFX leftMain = new WPI_TalonFX(Constants.Ports.Drive.leftMainID);
    public WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.Ports.Drive.leftFollowerID);
    public WPI_TalonFX rightMain = new WPI_TalonFX(Constants.Ports.Drive.rightMainID);
    public WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.Ports.Drive.rightFollowerID);

    public Solenoid shifter = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.Drive.shifterID);
    public boolean reversedHeading;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        reversedHeading = true;
        resetEncoders();
        resetOdometry(startPosition);
        motorConfig();

        SmartDashboard.putData(leftPIDController);
        SmartDashboard.putData(rightPIDController);
        // SmartDashboard.putData("TurnControl",turnController);

        SmartDashboard.putData("Field", simField);
        toggleGearLow();
    }

    public void motorConfig() {
        leftMain.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightMain.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftMain.setInverted(true);
        leftFollower.setInverted(InvertType.FollowMaster);

        leftFollower.follow(leftMain);
        rightFollower.follow(rightMain);

        leftMain.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightMain.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        leftMain.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                        Constants.DriveConstants.supplyCurrentThreshHold, 0.5));
        rightMain.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                        Constants.DriveConstants.supplyCurrentThreshHold, 0.5));

        leftFollower.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                        Constants.DriveConstants.supplyCurrentThreshHold, 0.5));
        rightFollower.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                        Constants.DriveConstants.supplyCurrentThreshHold, 0.5));

        leftMain.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);
        rightMain.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);

        leftFollower.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);
        rightFollower.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData("Gyro", gyro);
        SmartDashboard.putNumber("Right Position", getRightPostion());
        SmartDashboard.putNumber("Left Position", getLeftPostion());

        SmartDashboard.putBoolean("Swapcade Mode", reversedHeading);
        SmartDashboard.putString("Current Gear", getGearMode().toString());
        odometry.update(gyro.getRotation2d(), getLeftPostion(), getRightPostion());
        simField.setRobotPose(odometry.getPoseMeters());

    }

    /**
     * Sets the speeds of left and right main motors between -1.0 and 1.0
     * 
     * @param leftSpeed  The speed to set the left motor
     * @param rightSpeed The speed to set the right motor
     */
    public void setSpeedsPercent(double leftSpeed, double rightSpeed) {
        leftMain.set(ControlMode.PercentOutput, leftSpeed);
        rightMain.set(ControlMode.PercentOutput, rightSpeed);
    }

    // NOTE TO THOSE IN THE FUTURE: turnCommandR and turnCommandL ARE LAST MINUTE
    // AND INSANELY SKETCHY

    /**
     * Command to turn the robot a certain magnitude to the right
     * THIS IS VERY DUMB
     * @param turnValue Magnitude from 0 to 1.0
     */
    public void turnCommandR(double turnValue) {
        double left = turnValue;
        double right = -turnValue;
        setSpeedsPercent(left * Constants.DriveConstants.driveMultiplier,
                right * Constants.DriveConstants.driveMultiplier);
    }

    /**
     * Command to turn the robot a certain magnitude to the left
     * THIS IS ALSO VERY DUMB
     * @param turnValue magnitude from -1.0 to 0
     */
    public void turnCommandL(double turnValue) {
        double left = -turnValue;
        double right = turnValue;
        setSpeedsPercent(left * Constants.DriveConstants.driveMultiplier,
                right * Constants.DriveConstants.driveMultiplier);
    }

    /**
     * This  is an arcade drive method is to allow for linear motor input while following PID rotation
     */
    public void PIDLinearInput(double linear, double rotational)
    {
        double left = linear + rotational;
        double right = linear - rotational;

        setSpeedsPercent(left * Constants.DriveConstants.driveMultiplier,
        right * Constants.DriveConstants.driveMultiplier);
    }

    /**
     * Sets the voltages of the left and right motors.
     * 
     * @param leftVoltage  The voltage to set the left motor.
     * @param rightVoltage The voltage to set the right motor.
     */
    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftMain.setVoltage(leftVoltage);
        rightMain.setVoltage(rightVoltage);
    }

    /**
     * Sets the motors to certain closed loop velocities
     */
    public void setVelocities(double leftVelocity, double rightVelocity) {
        leftMain.set(ControlMode.Velocity, leftVelocity);
        rightMain.set(ControlMode.Velocity, leftVelocity);
    }

    /**
     * Resets odometry of robot to initial position
     * 
     * @param initPose Initial position of robot
     */
    public void resetOdometry(Pose2d initPose) {
        resetEncoders();
        gyro.reset();
        odometry.resetPosition(initPose, gyro.getRotation2d());

    }

    /**
     * Resets both motor encoders
     */
    public void resetEncoders() {
        rightMain.setSelectedSensorPosition(0);
        leftMain.setSelectedSensorPosition(0);
    }

    /**
     * Returns the kinematics of the robot
     */
    public DifferentialDriveKinematics getKinematics() {
        return Constants.AutoConstants.kinematics;
    }

    /**
     * Returns the current position of the robot on the field
     * 
     * @return Pose in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                ConversionMath.ticksToMeters(leftMain.getSelectedSensorVelocity(), getRatio()),
                ConversionMath.ticksToMeters(rightMain.getSelectedSensorVelocity(), getRatio()));
    }

    /**
     * Returns the current angle of the robot
     * 
     * @return The current gyroscopic angle
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    /**
     * Gets the displacement of the left encoder
     * 
     * @return Meters displaced by left side of robot
     */
    public double getLeftPostion() {
        return ConversionMath.ticksToMeters(leftMain.getSelectedSensorPosition(), getRatio());
    }

    /**
     * Gets the displacement of the right encoder
     * 
     * @return Meters displaced by right side of robot
     */
    public double getRightPostion() {
        return ConversionMath.ticksToMeters(rightMain.getSelectedSensorPosition(), getRatio());
    }

    /**
     * Gets PID controller for left side of robot.
     * 
     * @return lefPIDController.
     */
    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    /**
     * Gets PID controller for right side of the robot.
     * 
     * @return rightPIDController.
     */
    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    /**
     * Returns SimpleMotorFeedforward.
     * 
     * @return feeforward.
     */
    public SimpleMotorFeedforward getFeedForward() {
        return Constants.AutoConstants.feedforward;
    }

    /**
     * Sets high gear
     */
    public void toggleGearHigh() {
        shifter.set(true);
    }

    /**
     * Sets low gear
     */
    public void toggleGearLow() {
        shifter.set(false);
    }

    /**
     * Returns the current gearmode of the robot
     */
    public GearMode getGearMode() {
        return shifter.get() ? GearMode.high : GearMode.low;
    }

    /**
     * Returns current gear ratio
     * 
     * @return The double value of the gear ratio
     */
    public double getRatio() {
        return getGearMode().equals(GearMode.high) ? Constants.DriveConstants.highGearRatio
                : Constants.DriveConstants.lowGearRatio;
    }

    /**
     * Sets the current heading of the robot
     * 
     * @param reversed Heading to set
     */
    public void setHeading(boolean reversed) {
        reversedHeading = reversed;
    }

    /**
     * Returns whether the heading of the robot is reversed
     */
    public boolean isReversedHeading() {
        return reversedHeading;
    }
}