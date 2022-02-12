package org.team639.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;

public class Shoot extends CommandBase {
    Indexer indexer;

    long startTime;

    public Shoot(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() <= startTime + Constants.reverseIndexWhenShootingTime) {
            indexer.setIndexMotor(Constants.reverseIndexWhenShootingSpeed);
        } else if (System.currentTimeMillis() == startTime + Constants.reverseIndexWhenShootingTime) {

        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
