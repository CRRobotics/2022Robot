package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Acquisition extends SubsystemBase {

    TalonSRX acquisitionMotor = new TalonSRX(1);
    Solenoid rightPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //don't know what kind of piston this is
    Solenoid leftPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //idk if there are 2 pistons or not

    //maybe put this in Constants.java
    double motorSpeed = 0.5;

    public Acquisition() {
        acquisitionMotor.configFactoryDefault();
        acquisitionMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Puts acquisition down
     */
    public void acquisitionDown() {
        leftPiston.set(true);
        rightPiston.set(true);
    }

    /**
     * Puts acquisition up
     */
    public void acquisitionUp() {
        leftPiston.set(false);
        rightPiston.set(false);
    }

    /**
     * Spins cargo in
     */
    public void spinAcquisitionIn() {
        acquisitionMotor.set(ControlMode.PercentOutput, motorSpeed);
    }

    /**
     * Spins cargo out
     */
    public void spinAcquisitionOut() {
        acquisitionMotor.set(ControlMode.PercentOutput, motorSpeed*-1);
    }
}
