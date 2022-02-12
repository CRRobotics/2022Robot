package org.team639.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;

/**
 * StartAcquisition Command
 */
public class RunAcquisition extends CommandBase {

    Acquisition acquisition;

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
        acquisition.acquisitionDown();
    }

    @Override
    public void execute() {
        acquisition.spinAcquisitionIn(Constants.acquisitionMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        acquisition.stopAcquisitionMotor();
        acquisition.spinAcquisitionIn(Constants.acquisitionMotorSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
