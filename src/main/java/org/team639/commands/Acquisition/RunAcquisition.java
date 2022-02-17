package org.team639.commands.Acquisition;

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
        if(!acquisition.isAcquisitionDown())
            acquisition.acquisitionDown();
    }

    @Override
    public void execute() {
        acquisition.spinAcquisitionIn(acquisitionMotorSpeed);
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
