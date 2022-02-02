package org.team639.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;

public class AcquisitionCommand extends CommandBase {
    Acquisition acquisition;
    public AcquisitionCommand() {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ControllerWrapper.ControlController.getYButton()) {
            Acquisition.acquisitionDown();
            Acquisition.spinAcquisitionIn(Constants.acquisitionMotorSpeed);
        } else if (ControllerWrapper.ControlController.getXButton()) {
            Acquisition.acquisitionUp();
        }
        if (ControllerWrapper.ControlController.getBButton()) {
            Acquisition.spinAcquisitionOut(Constants.acquisitionMotorSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
