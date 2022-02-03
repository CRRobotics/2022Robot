// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import org.team639.commands.Drive.*;
import org.team639.subsystems.*;

import org.team639.lib.AutonMode;
import org.team639.lib.Constants;
import org.team639.lib.DriveLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  // Subsystem declaration
  private final DriveTrain driveTrain = new DriveTrain();

  // Command Declaration
  private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain);

  private static final SendableChooser<DriveLayout> driveMode;
  private static final SendableChooser<AutonMode> autoMode;

  private Pose2d basePose = new Pose2d();

  static {
    driveMode = new SendableChooser<>();
    driveMode.setDefaultOption("Arcade Standard", DriveLayout.Arcade);
    driveMode.addOption("CheesyDrive", DriveLayout.CheesyDrive);
    driveMode.addOption("Tank", DriveLayout.Tank);

    SmartDashboard.putData("Drive Layout", driveMode);
  }

  static 
  {
    autoMode = new SendableChooser<>();
    autoMode.setDefaultOption("AutoCross Line", AutonMode.crossLine);

    SmartDashboard.putData("Auto Mode", autoMode);
  }

  /**
   * Returns the current selected drive layout
   * @return The chosen drive layout
   */
  public static DriveLayout getDriveLayout() {
    return driveMode.getSelected();
  }

  /**
   * Returns the current selected auto layout
   * @return The chosen auto layout
   */
  public static AutonMode getAutonomousMode() {
    return autoMode.getSelected();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    defaultCommands();
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    driveTrain.resetOdometry(basePose);
    return null;
  }

  /**
   * Sets the default commands
   */
  public void defaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, joystickDrive);
  }


}
