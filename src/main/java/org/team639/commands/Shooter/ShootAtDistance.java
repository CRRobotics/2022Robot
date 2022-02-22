package org.team639.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team639.subsystems.DriveTrain;
import org.team639.subsystems.Indexer;
import org.team639.subsystems.Shooter;

import java.lang.reflect.Array;
import java.nio.charset.UnmappableCharacterException;
import java.util.ArrayList;
import java.util.HashMap;

public class ShootAtDistance extends CommandBase
{
    Indexer indexer;
    Shooter shooter;
    public ShootAtDistance(Indexer indexer, Shooter shooter)
    {
        this.indexer = indexer;
        this.shooter = shooter;
    }


}
