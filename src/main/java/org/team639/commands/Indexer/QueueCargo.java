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
            indexer.setIndexMotor(Constants.IndexConstants.indexQueueSpeed);
            
        }
        // while (acquisition.getAcquisitionIn()) { //while acquisition is running
        //     indexer.setIndexMotor(Constants.IndexConstants.indexQueueSpeed);
        //     if (!shooter.getExhaling()) { //if shooter is not running
        //         //have shooter motor spin in opposite direction
        //         if (indexer.topDetected() && indexer.bottomDetected()) {
        //             indexer.setIndexMotor(0);
        //             //stop shooter motor
        //             acquisition.stopAcquisitionMotor();
        //         }
        //     } else {
        //         //shoot shooter
        //     }
        // }
        // indexer.setIndexMotor(0);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
