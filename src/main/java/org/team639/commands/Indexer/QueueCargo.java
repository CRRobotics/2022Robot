package org.team639.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.subsystems.Indexer;

public class QueueCargo extends CommandBase {
    private Indexer indexer;
    //private Acquisition acquisition;
    //private Shooter shooter;

    public QueueCargo() {
        addRequirements(indexer);
        //adRequirements(acquisition);
        //addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        while () { //while acquisition is running
            indexer.setIndexMotor(Constants.indexMotorSpeed);
            if () { //if shooter is not running
                //have shooter motor spin in opposite direction
                if (indexer.topDetected() && indexer.bottomDetected()) {
                    indexer.setIndexMotor(0);
                    //stop shooter motor
                    //stop acquisition
                }
            } else {
                //shoot shooter
            }
        }
        indexer.setIndexMotor(0);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
