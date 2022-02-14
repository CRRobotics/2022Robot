// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import org.team639.auto.DriveRamsete;
import org.team639.auto.TrajectoryFactory;
import org.team639.commands.Drive.*;
import org.team639.controlboard.ControllerWrapper;
import org.team639.subsystems.*;

import org.team639.lib.AutonMode;
import org.team639.lib.Constants;
import org.team639.lib.DriveLayout;
import org.team639.lib.Songs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

  static SendableChooser<DriveLayout> driveMode = new SendableChooser<>();
  static SendableChooser<AutonMode> autoMode = new SendableChooser<>();
  static SendableChooser<String> songChooser = new SendableChooser<>();


  public static final TrajectoryFactory factory = new TrajectoryFactory("paths");

  static {
    driveMode.setDefaultOption("Arcade Standard", DriveLayout.Arcade);
    driveMode.addOption("Tank", DriveLayout.Tank);
    driveMode.addOption("CheesyDrive", DriveLayout.CheesyDrive);
    SmartDashboard.putData("Drive Layout", driveMode);
  }

  static {
    autoMode.setDefaultOption("AutoCross Line", AutonMode.crossLine);
    autoMode.addOption("3BallFender", AutonMode.ThreeBallFender);
    SmartDashboard.putData("Auto Mode", autoMode);
  }

  static {
    songChooser.setDefaultOption("Old Town Road", Constants.DJConstants.old_town_road);
    songChooser.addOption("Industry Baby", Constants.DJConstants.industry_baby);
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
   * Returns the current selected auto layout
   * 
   * @return The chosen auto layout
   */
  public static AutonMode getAutonomousMode() {
    return autoMode.getSelected();
  }
  /**
   * Returns the current song name loaded
   * @return Song to be chosen
   */
  public static String songChooser()
  {
    return songChooser.getSelected();
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
    ControllerWrapper.DriverButtonA.whenHeld(new DJRobot(driveTrain, 3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command auton;
    // switch(getAutonomousMode())
    // {
    //   default:
    //     auton = new AutoDriveForward(driveTrain, 2);
    //     break;
    //   case ThreeBallFender:
    //     auton = threeBallFender;
    //     break;
    // } 

    return new SequentialCommandGroup(new DriveRamsete(driveTrain,"bounce1"), new DriveRamsete(driveTrain,"bounce2"), new DriveRamsete(driveTrain,"bounce1"), new DriveRamsete(driveTrain, "bounce3"));
  }

  /**
   * Sets the default commands
   */
  public void defaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, joystickDrive);
  }

  
  

}