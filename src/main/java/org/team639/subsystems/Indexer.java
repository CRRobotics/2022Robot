// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.subsystems;

import org.team639.lib.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Indexer extends SubsystemBase {
    private VictorSPX indexMotor = new VictorSPX(Constants.Ports.Indexer.indexMotorID);
    private DigitalInput bottomSensor = new DigitalInput(0);
    private DigitalInput topSensor = new DigitalInput(1);

    //TODO configure top sensor
    //private ColorSensorV3 placeholderSensor = new ColorSensorV3(I2C.Port.kMXP);

    public Indexer() {
        indexMotor.configFactoryDefault();
        indexMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {

    }

    /**
     * Sets speed of motor from -1 to 1 
     * @param speed Speed to be set
     */
    public void setIndexMotor(double speed) {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public boolean bottomDetected()
    {
        return bottomSensor.get();
    }

    public boolean topDetected()
    {
        return topSensor.get();
    }
}