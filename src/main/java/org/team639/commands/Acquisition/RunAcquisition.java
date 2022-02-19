package org.team639.commands.Acquisition;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.subsystems.Acquisition;

/**
 * StartAcquisition Command
 */
public class RunAcquisition extends CommandBase {


    private Acquisition acquisition;
    private double acquisitionMotorSpeed;

    /**
     * Creates a new StartAcquisition
     * @param acquisition Acquisition to be used
     */
    public RunAcquisition(Acquisition acquisition, double acquisitionMotorSpeed) {
        this.acquisition = acquisition;
        this.acquisitionMotorSpeed = acquisitionMotorSpeed;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        System.out.println("RunAcquisition Initialized");
    }

    @Override
    public void execute() {
        acquisition.spinAcquisitionIn(acquisition.getAcquisitionSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopAcquisitionMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
