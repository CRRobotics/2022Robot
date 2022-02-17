package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team639.lib.Constants;

public class Acquisition extends SubsystemBase {

    CANSparkMax acquisitionMotorMain = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortMain, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax acquisitionMotorFollow = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortFollow, CANSparkMaxLowLevel.MotorType.kBrushless);
    Solenoid rightPiston = new Solenoid(PneumaticsModuleType.REVPH,Constants.Ports.Acquisition.acquisitionRightPistonID); //don't know what kind of piston this is
    Solenoid leftPiston = new Solenoid(PneumaticsModuleType.REVPH,Constants.Ports.Acquisition.acquisitionLeftPistonID); //idk if there are 2 pistons or not

    boolean acquisitionDown = false;

    public Acquisition() {
        acquisitionMotorMain.restoreFactoryDefaults();
        acquisitionMotorFollow.restoreFactoryDefaults();

        acquisitionMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotorFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);

        acquisitionMotorFollow.follow(acquisitionMotorMain);

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
        acquisitionMotorMain.set(speed);
    }

    /**
     * Spins cargo out
     */
    public void spinAcquisitionOut(double speed) {
        acquisitionMotorMain.set(speed*-1);
    }

    public void stopAcquisitionMotor() {
        acquisitionMotorMain.set(0);
    }

    public boolean getAcquisitionOut() {
        return acquisitionMotorMain.get() < 0;
    }

    public boolean getAcquisitionIn() {
        return acquisitionMotorMain.get() > 0;
    }
}