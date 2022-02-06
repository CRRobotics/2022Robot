package org.team639.commands.Acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;

public class AcquisitionCommand extends CommandBase { //better to use 1 command or 2 for acquisition?
    Acquisition acquisition;
    public AcquisitionCommand(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ControllerWrapper.ControlController.getYButton()) {
            acquisition.acquisitionDown();
            acquisition.spinAcquisitionIn(Constants.acquisitionMotorSpeed);
        } else if (ControllerWrapper.ControlController.getXButton()) {
            acquisition.acquisitionUp();
        }
        if (ControllerWrapper.ControlController.getBButton()) {
            acquisition.spinAcquisitionOut(Constants.acquisitionMotorSpeed);
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
