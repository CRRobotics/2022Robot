package org.team639.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.subsystems.Climb;

public class high extends CommandBase {
    private Climb climb;
    public high(Climb climb){
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
