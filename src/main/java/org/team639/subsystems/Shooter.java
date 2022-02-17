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

  private RelativeEncoder leftEncoder = mainMotor.getEncoder();
  private RelativeEncoder rightEncoder = followMotor.getEncoder();

  public double rightRPM;
  public double leftRPM;

  private long setpoint;

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
    rightRPM = rightEncoder.getVelocity();
    leftRPM = leftEncoder.getVelocity();

  }

  //turns the left motor on
  public void setSpeed(double speed){
    mainMotor.set(speed);
  }

  //Keeps the left motor at a target rpm
  public void setSpeedRPM(int setpoint){
    mainMotor.set(shooterPID.calculate(rightEncoder.getPosition(), setpoint));
  }

  public void actuatorMove(double pos){
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

}
