package org.team639.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase
{
    private VictorSPX winch1;
    private VictorSPX winch2;
    private  linearLeft;
    private  linearRight;

    public Climb()
    {
        winch1 = new VictorSPX();
        winch2 = new VictorSPX();
        linearLeft = new VictorSPX();
        linearLeft = new VictorSPX();
    }
    @Override
    public void periodic() {}
}
