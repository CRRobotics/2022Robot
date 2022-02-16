// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Shooter extends SubsystemBase {

  //Motor Controllers
  private CANSparkMax mainMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax followMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);

  private RelativeEncoder mainEncoder = mainMotor.getEncoder();
  private RelativeEncoder secondaryEncoder = followMotor.getEncoder();

  public double mainRPM;
  public double secondaryRPM;

  private PIDController shooterPID = new PIDController(0.0001, 0.001, 0);

  private Servo linearActuator1 = new Servo(0);
  private Servo linearActuator2 = new Servo(1);

  /** Creates a new Shooter. */
  public Shooter() {
    mainMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    mainMotor.setIdleMode(IdleMode.kCoast);
    followMotor.setIdleMode(IdleMode.kCoast);

    mainMotor.follow(followMotor);

    linearActuator1.setBounds(2.0,1.8,1.5,1.2,1.0);
    linearActuator1.setSpeed(1);

    linearActuator2.setBounds(2.0,1.8,1.5,1.2,1.0);
    linearActuator2.setSpeed(1);

  }

  @Override
  public void periodic() {
    mainRPM = mainEncoder.getVelocity() / 42;
    secondaryRPM = secondaryEncoder.getVelocity() / 42;
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
    linearActuator1.setPosition(pos);
    linearActuator2.setPosition(pos);
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
}