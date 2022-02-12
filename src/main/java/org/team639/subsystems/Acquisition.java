package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Acquisition extends SubsystemBase {

    TalonSRX acquisitionMotor = new TalonSRX(1);
    TalonSRX acquisitionMotor2 = new TalonSRX(1);
    TalonSRX acquisitionMotor3 = new TalonSRX(1); //idk if the other 2 are the right kind of motor
    Solenoid rightPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //don't know what kind of piston this is
    Solenoid leftPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //idk if there are 2 pistons or not

    public boolean acquisitionDown = false;

    public Acquisition() {
        acquisitionMotor.configFactoryDefault();
        acquisitionMotor.setNeutralMode(NeutralMode.Coast);

        acquisitionMotor2.configFactoryDefault();
        acquisitionMotor2.setNeutralMode(NeutralMode.Coast);
        acquisitionMotor2.follow(acquisitionMotor);

        acquisitionMotor3.configFactoryDefault();
        acquisitionMotor3.setNeutralMode(NeutralMode.Coast);
        acquisitionMotor3.follow(acquisitionMotor);
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
        acquisitionDown = true;
    }

    /**
     * Puts acquisition up
     */
    public void acquisitionUp() {
        leftPiston.set(false);
        rightPiston.set(false);
        acquisitionDown = false;
    }

    /**
     * Spins cargo in
     */
    public void spinAcquisitionIn(double speed) {
        acquisitionMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Spins cargo out
     */
    public void spinAcquisitionOut(double speed) {
        acquisitionMotor.set(ControlMode.PercentOutput, speed*-1);
    }

    public void stopAcquisitionMotor() {
        acquisitionMotor.set(ControlMode.PercentOutput, 0);
    }
}
