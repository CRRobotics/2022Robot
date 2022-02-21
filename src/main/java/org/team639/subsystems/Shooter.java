// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import org.team639.lib.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  private NetworkTableEntry shooterSpeed = tab.add("RPM", 5000).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  private NetworkTableEntry hoodPositioNetworkTableEntry = tab.add("HoodAngle", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  private NetworkTableEntry shooterSpeedPercent = tab.add("ShootPercent", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

  //Motor Controllers
  private CANSparkMax mainMotor = new CANSparkMax(Constants.Ports.Shooter.mainID, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax followMotor = new CANSparkMax(Constants.Ports.Shooter.followID, CANSparkMax.MotorType.kBrushless);

  private RelativeEncoder mainEncoder = mainMotor.getEncoder();

  public double mainRPM;
  public double secondaryRPM;

  //private PIDController shooterPID = new PIDController(0.0001, 0.001, 0);

  private SparkMaxPIDController maxController;

  private Servo mainLinearActuator = new Servo(Constants.Ports.Shooter.mainActuatorID);
  private Servo followLinearActuator = new Servo(Constants.Ports.Shooter.followActuatorID);

  /** Creates a new Shooter. */
  public Shooter() {
    mainMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    mainMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setIdleMode(IdleMode.kCoast);

    followMotor.follow(mainMotor, true);

    maxController = mainMotor.getPIDController();

    maxController.setP(Constants.ShooterConstants.shooterP);
    maxController.setI(Constants.ShooterConstants.shooterI);
    maxController.setD(Constants.ShooterConstants.shooterD);

    // mainMotor.burnFlash();
    // followMotor.burnFlash();

    mainLinearActuator.setBounds(1.8,1.8,1.5,1.2,1.0);
    mainLinearActuator.setSpeed(1);

    followLinearActuator.setBounds(1.8,1.8,1.5,1.2,1.0);
    followLinearActuator.setSpeed(1);

    setActuator(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getVelocity());
  }

/**
 * Sets shooter at certain speed
 * @param speed speed in percent from 1 to -1
 */  
  public void setSpeed(double speed){
    mainMotor.set(speed);
  }

  public double getSelectedHood()
  {
    return hoodPositioNetworkTableEntry.getDouble(0);
  }

  public double getSelectedRPM()
  {
    return shooterSpeed.getDouble(5000);
  }

  public double getSelectedSpeed()
  {
    return shooterSpeedPercent.getDouble(0);
  }

  /**
   * Sets shooter to certain rpm
   * @param setpoint RPM to set shooter
   */
  public void setSpeedRPM(double setpoint){
    maxController.setReference(setpoint, ControlType.kVelocity);
    

  }

  /**
   * Sets linear actuator to certain position
   * @param pos Position to be set between 0 and 1
   */
  public void setActuator(double pos){
    mainLinearActuator.setPosition(pos);
    followLinearActuator.setPosition(pos);
  }

  public double getVelocity()
  {
    return mainEncoder.getVelocity();
  }

  public double getActuatorPosition()
  {
    return mainLinearActuator.get();
  }

  public boolean getExhaling()
  {
    return mainMotor.get() > 0;
  }

  public boolean getInhaling()
  {
    return mainMotor.get() < 0;
  }

  public void stop()
  {
        mainMotor.set(0);
        followMotor.set(0);
  }

  /**
   * Bang bang control at a certain rpm. Make sure motors are on kCoast
   * @param rpm RPM Setpoint to be set
   */
  public void BangBangControl(double rpm)
  {
    if(mainMotor.getIdleMode().equals(IdleMode.kBrake))
      setCoast();
    if(mainEncoder.getVelocity() > rpm)
        setSpeed(0);
    else
        setSpeed(1);
  }

  public void setBrake()
  {
    mainMotor.setIdleMode(IdleMode.kBrake);
    followMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast()
  {
    mainMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setIdleMode(IdleMode.kCoast);
  }

}