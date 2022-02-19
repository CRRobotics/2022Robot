// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import org.team639.lib.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {

  //Motor Controllers
  private CANSparkMax mainMotor = new CANSparkMax(Constants.Ports.Shooter.mainID, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax followMotor = new CANSparkMax(Constants.Ports.Shooter.followID, CANSparkMax.MotorType.kBrushless);

  private RelativeEncoder mainEncoder = mainMotor.getEncoder();

  public double mainRPM;
  public double secondaryRPM;

  private PIDController shooterPID = new PIDController(0.0001, 0.001, 0);

  private Servo mainLinearActuator = new Servo(Constants.Ports.Shooter.mainActuatorID);
  private Servo followLinearActuator = new Servo(Constants.Ports.Shooter.followActuatorID);

  /** Creates a new Shooter. */
  public Shooter() {
    mainMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    mainMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setIdleMode(IdleMode.kCoast);

    mainMotor.follow(followMotor);

    mainLinearActuator.setBounds(2.0,1.8,1.5,1.2,1.0);
    mainLinearActuator.setSpeed(1);

    followLinearActuator.setBounds(2.0,1.8,1.5,1.2,1.0);
    followLinearActuator.setSpeed(1);

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

  /**
   * Sets shooter to certain rpm
   * @param setpoint RPM to set shooter
   */
  public void setSpeedRPM(int setpoint){
    mainMotor.set(shooterPID.calculate(mainEncoder.getPosition(), setpoint));
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