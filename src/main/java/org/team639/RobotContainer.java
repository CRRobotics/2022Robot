// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.team639.commands.ExampleCommand;
import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;
import org.team639.subsystems.ExampleSubsystem;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveTrain driveTrain = new DriveTrain();
  private static final SendableChooser<DriveLayout> driveMode;

  private String trajectoryJSON = "paths/Barrel_RacingTrue.wpilib.json";

  public enum DriveLayout {
    Arcade,
    CheesyDrive,
    Tank
  }

  static {
    driveMode = new SendableChooser<>();
    driveMode.setDefaultOption("Arcade Standard", DriveLayout.Arcade);
    driveMode.addOption("CheesyDrive", DriveLayout.CheesyDrive);
    driveMode.addOption("Tank", DriveLayout.Tank);

    SmartDashboard.putData("Drive Layout", driveMode);
  }

  /**
   * Returns the current selected drive layout
   * 
   * @return The chosen drive layout
   */
  public static DriveLayout getDriveLayout() {
    return driveMode.getSelected();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Loads a path from pathweaver into a Trajectory object
   * 
   * @return the trajectory loaded
   */
  public Trajectory loadConfig(String path) {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      Trajectory pathweaverTest = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return pathweaverTest;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }
    System.out.println("Warning: Path not Loaded");
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.kS,
            Constants.kV,
            Constants.kA),
        driveTrain.getKinematics(),
        12);

    // Set a trajectory config, setting constraints and stuff
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(driveTrain.getKinematics())
            .addConstraint(autoVoltageConstraint);

    Trajectory meter = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0)),
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

        RamseteCommand ramseteCommand = new RamseteCommand(
          meter,
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

    Trajectory pathweaverRunner = loadConfig(trajectoryJSON);

    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

}
