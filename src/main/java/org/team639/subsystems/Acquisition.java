package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team639.lib.Constants;

public class Acquisition extends SubsystemBase {

    static TalonSRX acquisitionMotor = new TalonSRX(1);
    static Solenoid rightPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //don't know what kind of piston this is
    static Solenoid leftPiston = new Solenoid(PneumaticsModuleType.REVPH,4); //idk if there are 2 pistons or not

    public static boolean acquisitionDown = false;

    public Acquisition() {
        //makes new acquisition
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
    public static void acquisitionDown() {
        leftPiston.set(true);
        rightPiston.set(true);
        acquisitionDown = true;
    }

    /**
     * Puts acquisition up
     */
    public static void acquisitionUp() {
        leftPiston.set(false);
        rightPiston.set(false);
        acquisitionDown = false;
    }

    /**
     * Spins cargo in
     */
    public static void spinAcquisitionIn(double speed) {
        acquisitionMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Spins cargo out
     */
    public static void spinAcquisitionOut(double speed) {
        acquisitionMotor.set(ControlMode.PercentOutput, speed*-1);
    }
}
