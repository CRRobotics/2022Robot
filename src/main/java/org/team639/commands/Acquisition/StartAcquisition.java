package org.team639.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;

/**
 * StartAcquisition Command
 */
public class StartAcquisition extends CommandBase {

    Acquisition acquisition;

    /**
     * Creates a new StartAcquisition
     * @param acquisition Acquisition to be used
     */
    public StartAcquisition(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {
        acquisition.acquisitionDown();
        acquisition.spinAcquisitionIn(Constants.acquisitionMotorSpeed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
