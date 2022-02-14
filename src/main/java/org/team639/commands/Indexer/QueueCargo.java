package org.team639.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

public class QueueCargo extends CommandBase {
    private Indexer indexer;
    private Acquisition acquisition;
    private Shooter shooter;

    public QueueCargo(Indexer indexer, Acquisition acquisition, Shooter shooter) {
        this.indexer = indexer;
        this.acquisition = acquisition;
        addRequirements(indexer);
        addRequirements(acquisition);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        indexer.setIndexMotor(0);
        shooter.setSpeed(0);
    }

    @Override
    public void execute() {
        if(acquisition.getAcquisitionIn())
        {
            indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
            shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setIndexMotor(0);
        shooter.setSpeed(0);
    }
}
