package org.team639.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private CANSparkMax bottomMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushed);
    private CANSparkMax topMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushed);
    private ColorSensorV3 topSensor = new ColorSensorV3(I2C.Port.kMXP); //these probably aren't right
    private ColorSensorV3 bottomSensor = new ColorSensorV3(I2C.Port.kMXP);

    double stationarySpeed = 0.128;

    public Indexer() {
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        topMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {

    }

    public void keepTopBallStationary() {
        bottomMotor.set(stationarySpeed);
        topMotor.set(stationarySpeed);
    }
}