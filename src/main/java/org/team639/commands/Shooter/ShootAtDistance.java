// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.Robot;
import org.team639.lib.Constants;
import org.team639.lib.math.AngleSpeed;
import org.team639.lib.math.ValueFromDistance;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

public class ShootAtDistance extends CommandBase {
    private Indexer indexer;
    private Shooter shooter;
    private Acquisition acquisition;

    private long startTime;
    private AngleSpeed shootAngleSpeed = ValueFromDistance.getAngleSpeed(Robot.getDistanceToTarget());

    public ShootAtDistance(Indexer indexer, Shooter shooter, Acquisition acquisition) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.acquisition = acquisition;
        addRequirements(indexer, shooter, acquisition);
    }

    @Override
    public void initialize() {
        shooter.setCoast();
        startTime = System.currentTimeMillis();
        shooter.setActuator(shootAngleSpeed.getAngle());
        shooter.setSpeed(Constants.ShooterConstants.reverseIndexSpeed);
        indexer.setIndexMotor(-Constants.IndexerConstants.indexMotorSpeed);

        acquisition.acquisitionNeutral();
    }

    @Override
    public void execute() {
        if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime)
        {
            indexer.setIndexMotor(0);
            shooter.setSpeedRPM(shootAngleSpeed.getSpeed());
        }
        if(System.currentTimeMillis() >= startTime + Constants.ShooterConstants.reverseIndexWhenShootingTime + Constants.ShooterConstants.spinUpTime)
        {
            shooter.setSpeedRPM(shootAngleSpeed.getSpeed());
            indexer.setIndexMotor(Constants.IndexerConstants.indexMotorSpeed);
            acquisition.spinAcquisition(acquisition.getAcquisitionSpeed());
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        indexer.setIndexMotor(0);
        acquisition.stopAcquisitionMotor();
        shooter.setBrake();
    }

    @Override
    public boolean isFinished() {
        if(System.currentTimeMillis() > startTime + Constants.ShooterConstants.pureShootingTime)
            return true;
        return false;
    }
}