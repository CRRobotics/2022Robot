package org.team639.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Indexer;

public class ReverseIndexer extends CommandBase {
    Indexer indexer;
    //private Acquisition acquisition;
    //private Shooter shooter;

    public ReverseIndexer() {
        addRequirements(indexer);
        //adRequirements(acquisition);
        //addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (indexer.topDetected()) {
            indexer.setIndexMotor(-1*Constants.indexMotorSpeed);
            if (indexer.bottomDetected()) {
                //set shooter to same speed as index motor
            }
            if () { //acquisition is running
                //spin acquisition backwards
            }
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        if () { //acquisition is running
            indexer.setIndexMotor(Constants.indexMotorSpeed);
        } else {
            indexer.setIndexMotor(0);
        }
    }
}
