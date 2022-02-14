package org.team639.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

public class ShootOpenLoop extends CommandBase {
    Indexer indexer;
    Shooter shooter;

    long startTime;

    public ShootOpenLoop(Indexer indexer, Shooter shooter, int rpm) {
        this.indexer = indexer;
        this.shooter = shooter;
        addRequirements(indexer);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() < startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime) {
            indexer.setIndexMotor(Constants.ShooterConstants.reverseIndexSpeed);
        } else if (System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime) {
            shooter.setSpeed(Constants.ShooterConstants.shootHighSpeed);
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
