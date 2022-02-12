package org.team639.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team639.auto.DriveRamsete;
import org.team639.subsystems.DriveTrain;

public class FiveBallRoutine extends SequentialCommandGroup {
    public FiveBallRoutine(DriveTrain drivetrain) {
        addCommands(
                new DriveRamsete(drivetrain, "1")
        );
    }
}
