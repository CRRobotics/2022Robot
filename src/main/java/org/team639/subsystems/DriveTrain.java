// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import org.team639.auto.TrajectoryFactory;
import org.team639.lib.Constants;
import org.team639.lib.GearMode;
import org.team639.lib.math.ConversionMath;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

    // Gyroscope and initial pose
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    private Pose2d startPosition = new Pose2d(new Translation2d(0, 0), getHeading());

    // Differential Control Systems
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

    // Independent left and right PID controllers
    PIDController leftPIDController = new PIDController(.001, 0, 0);
    PIDController rightPIDController = new PIDController(.001, 0, 0);

    // Talon motor controllers
    private TalonFX leftMain = new TalonFX(Constants.Ports.Drive.leftMainID);
    private TalonFX leftFollower = new TalonFX(Constants.Ports.Drive.leftFollowerID);
    private TalonFX rightMain = new TalonFX(Constants.Ports.Drive.rightMainID);
    private TalonFX rightFollower = new TalonFX(Constants.Ports.Drive.rightFollowerID);

    //TODO: Fix the solenoid controls
    //private Solenoid shifter = new Solenoid(PneumaticsModuleType.REVPH, Constants.shifterID);

    TrajectoryFactory factory = new TrajectoryFactory("paths");

    public static GearMode currGear;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        resetOdometry(startPosition);
        motorConfig();
       // this.toggleGearHigh();
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

        leftMain.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                Constants.DriveConstants.supplyCurrentThreshHold, 0.5));
        rightMain.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                Constants.DriveConstants.supplyCurrentThreshHold, 0.5));

        leftFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                Constants.DriveConstants.supplyCurrentThreshHold, 0.5));
        rightFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.DriveConstants.supplyCurrentLimiter,
                Constants.DriveConstants.supplyCurrentThreshHold, 0.5));

        leftMain.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);
        rightMain.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);
        
        leftFollower.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);
        rightFollower.configOpenloopRamp(Constants.DriveConstants.kDriveRampSeconds);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());
        SmartDashboard.putNumber("Right Position", getRightPostion());
        SmartDashboard.putNumber("Left Position", getLeftPostion());
        
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

    // TODO: Fix this and set up all SmartDashboard values correctly

    /**
     * Sets the voltages of the left and right motors.
     * 
     * @param leftVoltage  The voltage to set the left motor.
     * @param rightVoltage The voltage to set the right motor.
     */
    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftMain.set(ControlMode.Current, leftVoltage);
        rightMain.set(ControlMode.Current, leftVoltage);
    }

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
        odometry.resetPosition(initPose, getHeading());
        gyro.reset();

    }

      /**
   * Generates a Ramsete command
   * 
   * @return the generated command
   */
  public RamseteCommand ramseteGenerator(Trajectory pathRunner) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        pathRunner,
        this::getPose,
        new RamseteController(2.0, 0.7),
        Constants.AutoConstants.feedforward,
        Constants.AutoConstants.kinematics,
        this::getWheelSpeeds,
        leftPIDController,
        rightPIDController,
        this::setVoltages,
        this);
    return ramseteCommand;
  }

  public RamseteCommand FenderToFender = this.ramseteGenerator(factory.getTrajectory("3BallFender"));


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

    // TODO: Convert ticks to meters per second
    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMain.getSelectedSensorVelocity(),
                rightMain.getSelectedSensorVelocity());
    }

    /**
     * Returns the current angle of the robot
     * 
     * @return The current gyroscopic angle
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    /**
     * Gets the displacement of the left encoder
     * 
     * @return Meters displaced by left side of robot
     */
    public double getLeftPostion() {
        // return leftMain.getSelectedSensorPosition(0) * Constants.driveTrainGearRatio
        // * (Units.inchesToMeters(6)*Math.PI);
        return ConversionMath.ticksToMeters(leftMain.getSelectedSensorPosition(0));
    }

    /**
     * Gets the displacement of the right encoder
     * 
     * @return Meters displaced by right side of robot
     */
    public double getRightPostion() {
        // return rightMain.getSelectedSensorPosition(0) * Constants.driveTrainGearRatio
        // * (Units.inchesToMeters(6)*Math.PI);
        return ConversionMath.ticksToMeters(rightMain.getSelectedSensorPosition(0));

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

    // /**
    //  * Sets high gear
    //  */
    // public void toggleGearHigh() {
    //     shifter.set(true);
    //     currGear = GearMode.high;
    // }

    // /**
    //  * Sets low gear
    //  */
    // public void toggleGearLow() {
    //     shifter.set(false);
    //     currGear = GearMode.low;
    // }

    public static GearMode getGear()
    {
        return currGear;
    }
    
}
