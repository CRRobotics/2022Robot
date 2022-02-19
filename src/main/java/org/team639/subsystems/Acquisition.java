package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team639.lib.Constants;
import org.team639.lib.states.AcquisitionPosition;

public class Acquisition extends SubsystemBase {
    private ShuffleboardTab tab = Shuffleboard.getTab("Acquisition");
    private NetworkTableEntry acquisitionSpeed = tab.add("AcqSpeed", 0.5).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    CANSparkMax acquisitionMotorMain = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortMain, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax acquisitionMotorFollow = new CANSparkMax(Constants.Ports.Acquisition.acquisitionPortFollow, CANSparkMaxLowLevel.MotorType.kBrushless);
    Solenoid extend = new Solenoid(PneumaticsModuleType.REVPH,Constants.Ports.Acquisition.acquisitionExtend); //don't know what kind of piston this is
    Solenoid retract = new Solenoid(PneumaticsModuleType.REVPH,Constants.Ports.Acquisition.acquisitionRetract); //idk if there are 2 pistons or not

    AcquisitionPosition acqPos = AcquisitionPosition.down;

    public Acquisition() {
        acquisitionMotorMain.restoreFactoryDefaults();
        acquisitionMotorFollow.restoreFactoryDefaults();
        acquisitionMotorMain.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotorFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
        acquisitionMotorFollow.follow(acquisitionMotorMain, true);
        acquisitionNeutral();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Acquisition Position", acqPos().toString());
    }

    /**
     * Puts acquisition down
     */
    public void acquisitionNeutral() {
        // if(!isAcquisitionDown())
        // {
        //     leftPiston.set(true);
        //     rightPiston.set(true);
        //     acquisitionDown = true;
        // }
        extend.set(true);
        retract.set(false);

    }

    /**
     * Puts acquisition up
     */
    public void acquisitionDown() {
        // if(isAcquisitionDown())
        // {
        //     leftPiston.set(false);
        //     rightPiston.set(false);
        //     acquisitionDown = false;
        // }
        extend.set(false);
        retract.set(true);
    }

    public void acquisitionUp()
    {
        extend.set(false);
        retract.set(false);
    }

    public AcquisitionPosition acqPos()
    {
        if(extend.get() == false && retract.get() == true)
            return AcquisitionPosition.down;
        else if(extend.get() == true && retract.get() == false)
            return AcquisitionPosition.up;
        return AcquisitionPosition.neutral;
    }

    public double getAcquisitionSpeed()
    {
        return acquisitionSpeed.getDouble(0.5);
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