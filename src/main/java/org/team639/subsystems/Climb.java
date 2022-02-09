package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase
{
    private VictorSPX winch1;
    private VictorSPX winch2;
    private CANSparkMax linearLeft;
    private CANSparkMax linearRight;
    private Object linearEncoder; //TODO: replace placeholder
    private Object winchEncoder; //TODO: replace placeholder
       //TODO: Find correct device IDs for the motors
    public Climb()
    {
        winch1 = new VictorSPX(0);
        winch2 = new VictorSPX(0);
        linearLeft = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        linearLeft = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
    @Override
    public void periodic() {}

    public void driveWinch(double position)
    {
        while (winchEncoder != position) {
            winch1.set((winchEncoder < position)? 1:-1);
            winch2.set((winchEncoder < position)? 1:-1);
        }
    }
    public void driveLinear(double position)
    {
        while (linearEncoder != position) {
            linearLeft.set((linearEncoder < position)? 1:-1);
            linearRight.set((linearEncoder < position)? 1:-1);
        }
    }
}
