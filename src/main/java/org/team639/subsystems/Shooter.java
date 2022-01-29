// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  //Motor Controllers
  private CANSparkMax topMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax botMotor = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);

  //Time in seconds
  double timer;
  double timer1;

  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    timer += 0.02;

  }

  //turns the top motor on
  public void toggleTopOn(double speed){
    topMotor.set(speed);
  }

  //turns the bottom motor on
  public void toggleBotOn(double speed){
    botMotor.set(speed);
  }


  // //spins the top motor at a specified speed for a specified time
  // public void spinTopMotorTimed(double speed, double time){
  //   timer = 0;
  //   while(timer < time){
  //     topMotor.set(speed);
  //   }
  //   if(timer >= time){
  //     topMotor.set(0);
  //   }
  // }

  // //spins the bottom motor at a specified speed for a specified time
  // public void spinBotMotorTimed(double speed, double time){
  //   timer1 = 0;
  //   while(timer1 < time){
  //     botMotor.set(speed);
  //   }
  //   if(timer1 >= time){
  //     botMotor.set(0);
  //   }
  // }
}
