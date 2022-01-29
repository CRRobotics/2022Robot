// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  //Motor Controllers
  private CANSparkMax leftMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightMotor.getEncoder();

  public double rightRPM;
  public double leftRPM;


  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    rightRPM = rightEncoder.getVelocity()/42;
    leftRPM = leftEncoder.getVelocity()/42;

  }

  //turns the left motor on
  public void toggleleftOn(double speed){
    leftMotor.set(speed);
  }

  //turns the right motor on
  public void togglerightOn(double speed){
    rightMotor.set(speed);
  }

  //Keeps the left motor at a target rpm
  public void maintainLeftRPM(double targetRPM){
    if(leftRPM >= targetRPM){
      leftMotor.set(0);
    }
    else{
      leftMotor.set(1);
    }  
  }

  //Keeps the right motor at a target rpm
  public void maintainRightRPM(double targetRPM){
    if(rightRPM >= targetRPM){
      rightMotor.set(0);
    }
    else{
      rightMotor.set(1);
    }
  }

}
