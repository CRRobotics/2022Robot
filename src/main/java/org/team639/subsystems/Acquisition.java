package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team639.lib.Constants;

public class Acquisition extends SubsystemBase {

    CANSparkMax acquisitionMotor = new CANSparkMax(Constants.Ports.Acquisition.acquisitionMotor1port, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax acquisitionMotor2 = new CANSparkMax(Constants.Ports.Acquisition.acquisitionMotor2port, CANSparkMaxLowLevel.MotorType.kBrushless);
    Solenoid rightPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //don't know what kind of piston this is
    Solenoid leftPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //idk if there are 2 pistons or not

    boolean acquisitionDown = false;

    public Acquisition() {
        acquisitionMotor.restoreFactoryDefaults();
        acquisitionMotor2.restoreFactoryDefaults();
        acquisitionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        acquisitionMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotor2.follow(acquisitionMotor);

        acquisitionUp();
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
     * Determines if the acquistion is deployed
     * @return acquistionDown
     */
    public boolean isAcquisitionDown()
    {
        return acquisitionDown;
    }

    /**
     * Spins cargo in
     */
    public void spinAcquisitionIn(double speed) {
        acquisitionMotor.set(speed);
    }

    /**
     * Spins cargo out
     */
    public void spinAcquisitionOut(double speed) {
        acquisitionMotor.set(speed*-1);
    }

    public void stopAcquisitionMotor() {
        acquisitionMotor.set(0);
    }

    public boolean getAcquisitionOut() {
        return acquisitionMotor.get() < 0;
    }

    public boolean getAcquisitionIn() {
        return acquisitionMotor.get() > 0;
    }
}
