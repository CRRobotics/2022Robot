// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      //return pose;
      return odometry.getPoseMeters();
  }
  
  /**
   * Returns the current angle of the robot
   * @return The current gyroscopic angle
   */
  public Rotation2d getHeading()
  {
      return Rotation2d.fromDegrees(gyro.getAngle());
  }
}
