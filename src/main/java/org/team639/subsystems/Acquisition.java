package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team639.lib.Constants;
import org.team639.lib.states.AcquisitionPosition;

public class Acquisition extends SubsystemBase {

    CANSparkMax acquisitionMotorMain = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortMain,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax acquisitionMotorFollow = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortFollow,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    Solenoid extend = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.Acquisition.acquisitionExtend);
    //Solenoid retract = new Solenoid(PneumaticsModuleType.REVPH, Constants.Ports.Acquisition.acquisitionRetract);

    AcquisitionPosition acqPos = AcquisitionPosition.up;

    public Acquisition() {
        acquisitionMotorMain.restoreFactoryDefaults();
        acquisitionMotorFollow.restoreFactoryDefaults();
        acquisitionMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotorFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotorFollow.follow(acquisitionMotorMain, true);
        acquisitionDown();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Acquisition Position", acqPos().toString());
    }

    // /**
    //  * Puts acquisition in neutral
    //  */
    // public void acquisitionNeutral() {
    //     if (!(acqPos().equals(AcquisitionPosition.neutral))) {
    //         extend.set(true);
    //         retract.set(false);
    //     }
    //     acqPos = AcquisitionPosition.neutral;
    // }

    /**
     * Puts acquisition down
     */
    public void acquisitionDown() {
        if (!(acqPos().equals(AcquisitionPosition.down))) {
            extend.set(true);
            //extend.set(false);
            //retract.set(true);
        }
        acqPos = AcquisitionPosition.down;

    }

    /**
     * Puts acquisition up
     */
    public void acquisitionUp() {
        if (!(acqPos().equals(AcquisitionPosition.up))) {
            extend.set(false);
            //retract.set(false);
        }
        acqPos = AcquisitionPosition.up;

    }

    /**
     * Return the current acquisition position
     */
    public AcquisitionPosition acqPos() {
        return acqPos;
    }

    /**
     * Spins acquisition
     * 
     * @param speed from -1.0 to 1.0
     */
    public void spinAcquisition(double speed) {
        if(acqPos() != AcquisitionPosition.up)
            acquisitionMotorMain.set(MathUtil.clamp(speed, -1.0, 1.0));
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