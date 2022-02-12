package org.team639.subsystems;


import com.revrobotics.ColorSensorV3;
import org.team639.lib.Constants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Indexer extends SubsystemBase {
    private VictorSPX indexMotor = new VictorSPX(Constants.Ports.Indexer.indexMotorID);
    private ColorSensorV3 bottomSensor = new ColorSensorV3(I2C.Port.kMXP);

    double stationarySpeed = 0.128; 

    public Indexer() {
        indexMotor.configFactoryDefault();
        indexMotor.setNeutralMode(NeutralMode.Brake);
        bottomSensor.configureProximitySensor(ColorSensorV3.ProximitySensorResolution.kProxRes11bit, ColorSensorV3.ProximitySensorMeasurementRate.kProxRate50ms);

    }

    @Override
    public void periodic() {

    }

    /**
     * Sets speed of motor from -1 to 1 
     * @param speed Speed to be set
     */
    public void setIndexMotor(double speed)
    {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    public int getProximity()
    {
        return bottomSensor.getProximity();
    }
}