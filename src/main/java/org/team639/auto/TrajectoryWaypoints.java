// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.auto;

import java.io.IOException;
import java.nio.file.Path;

import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/** 
 * Series of trajectories to be used in autonomous routines
 */
public class TrajectoryWaypoints {
    private DriveTrain driveTrain;
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.kS,
            Constants.kV,
            Constants.kA),
        driveTrain.getKinematics(),
        12);

    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(driveTrain.getKinematics())
            .addConstraint(autoVoltageConstraint);


    //PATHS
    private String Fender3Ball = "paths/3BallFender.wpilib.json";
    private String TestPath = "paths/Test Path.wpilib.json";


    public TrajectoryWaypoints(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;
    }

    /**
   *Generates a Ramsete command
   * @return the generated command
   */
  public RamseteCommand ramseteGenerator(Trajectory pathRunner) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        pathRunner,
        driveTrain::getPose,
        new RamseteController(2.0, 0.7),
        driveTrain.getFeedForward(),
        driveTrain.getKinematics(),
        driveTrain::getWheelSpeeds,
        driveTrain.getLeftPIDController(),
        driveTrain.getRightPIDController(),
        driveTrain::setVoltages,
        driveTrain
    );
    return ramseteCommand;
  }

    /**
   * Loads a path from pathweaver into a Trajectory object
   * 
   * @return the trajectory loaded
   */
  public Trajectory loadConfig(String path) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      Trajectory runner = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return runner;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }
    System.out.println("Warning: Path not Loaded");
    return null;
  }

  //Test of Ramsete. Should move 2 meters forward
  public RamseteCommand Test = ramseteGenerator(loadConfig(TestPath));


  //3 ball autonomous that starts from the fender and ends at the fender
  public RamseteCommand FenderToFender = ramseteGenerator(loadConfig(Fender3Ball));

}
