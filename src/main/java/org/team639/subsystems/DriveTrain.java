// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.team639.lib.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {

//Gyroscope and initial pose
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  private Pose2d startPosition = new Pose2d(new Translation2d(0, 0), getHeading());

  //Differential Control Systems
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.chassisWidth);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  
  //Independent left and right PID controllers
  PIDController leftPIDController = new PIDController(.0001, 0, 0);
  PIDController rightPIDController = new PIDController(.0001, 0, 0);

  //Talon motor controllers
  private TalonFX leftMain = new TalonFX(Constants.leftMainID);
  private TalonFX leftFollower = new TalonFX(Constants.leftFollowerID);
  private TalonFX rightMain = new TalonFX(Constants.rightMainID);
  private TalonFX rightFollower = new TalonFX(Constants.rightFollowerID);



  /** Creates a new DriveTrain. */
  public DriveTrain() 
  {
    resetOdometry(startPosition);
    motorConfig();
  }

  public void motorConfig()
  {
    leftMain.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMain.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftFollower.follow(leftMain);
    rightFollower.follow(rightMain);

    leftMain.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMain.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMain.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      Constants.supplyCurrentLimiter,                Constants.supplyCurrentThreshHold,                0.5));
    rightMain.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      Constants.supplyCurrentLimiter,                Constants.supplyCurrentThreshHold,                0.5));

    leftFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      Constants.supplyCurrentLimiter,                Constants.supplyCurrentThreshHold,                0.5));
    rightFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      Constants.supplyCurrentLimiter,                Constants.supplyCurrentThreshHold,                0.5));

    leftMain.configOpenloopRamp(Constants.kDriveRampSeconds);
    rightMain.configOpenloopRamp(Constants.kDriveRampSeconds);
    leftFollower.configOpenloopRamp(Constants.kDriveRampSeconds);
    rightFollower.configOpenloopRamp(Constants.kDriveRampSeconds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());
    SmartDashboard.putNumber("Right Encoder Position", getRightPostion());
    SmartDashboard.putNumber("Right Encoder Position", getLeftPostion());

  }

  /** 
  * Sets the speeds of left and right main motors between -1.0 and 1.0
  * @param leftSpeed The speed to set the left motor
  * @param rightSpeed The speed to set the right motor
  */
 public void setSpeedsPercent(double leftSpeed, double rightSpeed)
 {
   leftMain.set(ControlMode.PercentOutput, leftSpeed); 
   rightMain.set(ControlMode.PercentOutput, rightSpeed); 
 }

//TODO: Fix this and set up all SmartDashboard values correctly

  /**
   * Sets the voltages of the left and right motors.
   * @param leftVoltage The voltage to set the left motor.
   * @param rightVoltage The voltage to set the right motor.
   */
  public void setVoltages(double leftVoltage, double rightVoltage)
  {
      leftMain.set(ControlMode.Current, leftVoltage);
      rightMain.set(ControlMode.Current, leftVoltage);
  }

  public void setVelocities(double leftVelocity, double rightVelocity)
  {
      leftMain.set(ControlMode.Velocity, leftVelocity);
      rightMain.set(ControlMode.Velocity, leftVelocity);
  }

  /**
   * Resets odometry of robot to initial position
   * @param initPose Initial position of robot
   */
  public void resetOdometry(Pose2d initPose)
  {
      gyro.reset();
      odometry.resetPosition(initPose, getHeading());
  }

  /**
   * Returns the kinematics of the robot
   */
  public DifferentialDriveKinematics getKinematics()
  {
      return kinematics;
  }
  
  /**
   * Returns the current position of the robot on the field
   * @return Pose in meters
   */
  public Pose2d getPose()
  {
      return odometry.getPoseMeters();
  }

  //TODO: Convert ticks to meters per second
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
      return new DifferentialDriveWheelSpeeds(leftMain.getSelectedSensorVelocity(), rightMain.getSelectedSensorVelocity());
  }
  
  /**
   * Returns the current angle of the robot
   * @return The current gyroscopic angle
   */
  public Rotation2d getHeading()
  {
      return Rotation2d.fromDegrees(gyro.getAngle());
  }
  /**
   * Gets the displacement of the left encoder
   * @return Meters displaced by left side of robot
   */
  public double getLeftPostion()
  {
      return leftMain.getSelectedSensorPosition(0) * Constants.driveTrainGearRatio * (Units.inchesToMeters(6)*Math.PI);
  }

  /**
   * Gets the displacement of the right encoder
   * @return Meters displaced by right side of robot
   */
  public double getRightPostion()
  {
      return rightMain.getSelectedSensorPosition(0)  * Constants.driveTrainGearRatio * (Units.inchesToMeters(6)*Math.PI);
  }

  /**
   * Gets PID controller for left side of robot.
   * @return lefPIDController.
   */
  public PIDController getLeftPIDController()
  {
      return leftPIDController;
  }
  
  /**
   * Gets PID controller for right side of the robot.
   * @return rightPIDController.
   */
  public PIDController getRightPIDController()
  {
      return rightPIDController;
  }
  
  /**
   * Returns SimpleMotorFeedforward.
   * @return feeforward.
   */
  public SimpleMotorFeedforward getFeedForward()
  {
      return feedforward;
  }
}
