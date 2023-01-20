// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.lib.Constants;
import org.team639.lib.math.AngleSpeed;
import org.team639.lib.math.ValueFromDistance;
import org.team639.subsystems.Acquisition;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.LED;
import org.team639.subsystems.Shooter;

public class AdjustHood extends CommandBase {
    private Indexer indexer;
    private Shooter shooter;
    private Acquisition acquisition;
    private LED led;
    private double adjustHood;

    private long startTime;
    private AngleSpeed shootAngleSpeed;


    public AdjustHood(double adjustHood, Shooter shooter) {
        this.shooter = shooter;
        this.adjustHood = adjustHood;
        // shootAngleSpeed = ValueFromDistance.getAngleSpeed(distance);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
       
        shooter.setActuator(adjustHood);
    }

    @Override
    public void execute() {
        
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
        return true;
    }
}