package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase
{
    private VictorSPX winch1;
    private VictorSPX winch2;
    private CANSparkMax linearLeft;
    private CANSparkMax linearRight;

    public Climb()
    {
        winch1 = new VictorSPX();
        winch2 = new VictorSPX();
        linearLeft = new CANSparkMax();
        linearLeft = new CANSparkMax();
    }
    @Override
    public void periodic() {}

    public void extendWinch()
    {

    }
    public void turnLinear()
    {
        while()
        {}
    }
}
