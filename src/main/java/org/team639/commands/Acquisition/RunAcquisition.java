package org.team639.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;

/**
 * StartAcquisition Command
 */
public class RunAcquisition extends CommandBase {


    private Acquisition acquisition;

    /**
     * Creates a new StartAcquisition
     * @param acquisition Acquisition to be used
     */
    public RunAcquisition(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        System.out.println("RunAcquisition Initialized");
    }

    @Override
    public void execute() {
        acquisition.spinAcquisition(Constants.AcquisitionConstants.acquisitionSpeed);
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
