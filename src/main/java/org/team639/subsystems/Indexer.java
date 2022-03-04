package org.team639.subsystems;

import org.team639.lib.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Indexer extends SubsystemBase {
    private VictorSPX indexMotor = new VictorSPX(Constants.Ports.Indexer.indexMotorID);

    public Indexer() {
        indexMotor.configFactoryDefault();
        indexMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {

    }

    /**
     * Sets speed of motor from -1 to 1
     * 
     * @param speed Speed to be set
     */
    public void setIndexMotor(double speed) {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

}