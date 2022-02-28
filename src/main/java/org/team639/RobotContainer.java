// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import org.team639.auto.*;
import org.team639.commands.Acquisition.*;
import org.team639.commands.Drive.*;
import org.team639.commands.Indexer.*;
import org.team639.commands.Shooter.*;
import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.math.AngleSpeed;
import org.team639.subsystems.*;
import org.team639.lib.states.*;
import org.team639.lib.Constants;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();
  private final Acquisition acquisition = new Acquisition();

  // Command Declaration
  AutonomousRoutines auton = new AutonomousRoutines();
  //Drive
  private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain);
  private final ToggleGears shiftGears = new ToggleGears(driveTrain);
  private final ReverseHeading swap = new ReverseHeading(driveTrain);
  private final RotateToTarget autoAim = new RotateToTarget(driveTrain);

  //Acquisition
  private final ToggleAcquisition toggleAcquisition = new ToggleAcquisition(acquisition);

  //Indexer
  private final SpitCargo eject = new SpitCargo(shooter, indexer, acquisition);
  private final ManualIndexer index = new ManualIndexer(shooter, indexer, acquisition);

  //Shooter
  private final ShootClosedLoop fenderShot = new ShootClosedLoop(indexer, shooter, acquisition, Constants.ShooterConstants.fenderRPM, Constants.ShooterConstants.fenderAngle);
  private final AutoShootAtDistance autoShoot = new AutoShootAtDistance(indexer, shooter, acquisition);
  
  public static SendableChooser<DriveLayout> driveMode = new SendableChooser<>();
  public static SendableChooser<AutonMode> autoMode = new SendableChooser<>();
  public static SendableChooser<String> songChooser = new SendableChooser<>();

  public static final TrajectoryFactory factory = new TrajectoryFactory("paths");

  static
  {
//    Constants.ShooterConstants.shootMap.put(,new AngleSpeed(.9,3000));
    Constants.ShooterConstants.shootMap.put(1.92, new AngleSpeed(.7,3200));
    Constants.ShooterConstants.shootMap.put(2.77, new AngleSpeed(.65, 3400));
    Constants.ShooterConstants.shootMap.put(4.27, new AngleSpeed(.5, 3800));
    Constants.ShooterConstants.shootMap.put(5.46, new AngleSpeed(.4, 4150));
    Constants.ShooterConstants.shootMap.put(6.85, new AngleSpeed(.15, 4600));
  }

  static {
    driveMode.setDefaultOption("Arcade Standard", DriveLayout.Arcade);
    driveMode.addOption("Arcade InversedK", DriveLayout.ArcadeInverseK);
    driveMode.addOption("Curvature", DriveLayout.CurvatureDrive);
    driveMode.addOption("Tank", DriveLayout.Tank);

    SmartDashboard.putData("Drive Layout", driveMode);
  }

  static {
    autoMode.setDefaultOption("AutoCross Line", AutonMode.ForwardTest);
    autoMode.addOption("2BallAuto", AutonMode.TwoBall);
    autoMode.addOption("4BallAuto", AutonMode.FourBall);
    autoMode.addOption("Shoot", AutonMode.Shoot);
    autoMode.addOption("BounceTest", AutonMode.BounceTest);
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
    //Driver
    ControllerWrapper.DriverRightBumper.whenPressed(shiftGears);
    ControllerWrapper.DriverLeftBumper.whenPressed(swap);
    ControllerWrapper.DriverButtonX.whenPressed(autoAim);

    //Controller
    ControllerWrapper.ControlRightBumper.whenHeld(index);
    ControllerWrapper.ControlLeftBumper.whenHeld(eject);

    ControllerWrapper.ControlButtonY.whenHeld(fenderShot); 
    ControllerWrapper.ControlButtonB.whenPressed(toggleAcquisition);
    ControllerWrapper.ControlButtonX.whenPressed(autoShoot);

    //RESET BUTTONS
    ControllerWrapper.ControlDPadUp.whenPressed(new ToggleActuator(shooter));
    ControllerWrapper.ControlDPadDown.whenPressed(new Autorotate(driveTrain, 90, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto;
    switch(getAutonomousMode())
    {
      default:
            auto = auton.forwardTest;
        break;
      case ForwardTest:
        auto = auton.forwardTest;
        break;
      // case Shoot:
      //   auto = shootOpen;
      //   break;
      case TwoBall:
        auto = auton.twoBallAutonomous;
        break;
      case FourBall:
        auto = auton.fourBallAutonomousAndRohitsJumper;
        break;
      case BounceTest:
        auto = auton.bounceTest;
        break;
    } 
    return auto;
  }

  /**
   * Sets the default commands
   */
  public void defaultCommands() {
   CommandScheduler.getInstance().setDefaultCommand(driveTrain, joystickDrive);
 // CommandScheduler.getInstance().setDefaultCommand(shooter, manualAim);
  }

  class TeleopRoutines
  {
    final SequentialCommandGroup aimbotshot = new SequentialCommandGroup(new RotateToTarget(driveTrain), new AutoShootAtDistance(indexer, shooter, acquisition));
    
  }
  
  class AutonomousRoutines
  {
    //Testing commands
    final SequentialCommandGroup forwardTest = new SequentialCommandGroup(
      new DriveRamsete(driveTrain, "TestPath")
    );
    final SequentialCommandGroup bounceTest = new SequentialCommandGroup(
      new DriveRamsete(driveTrain,"bounce1"), 
      new DriveRamsete(driveTrain,"bounce2"), 
      new DriveRamsete(driveTrain,"bounce1"), 
      new DriveRamsete(driveTrain, "bounce3")
    ); 
    //Start Position: 7.635, 1.786 - Facing bottom team ball, bumpers against the tarmac edge
    final SequentialCommandGroup fourBallAutonomousAndRohitsJumper = new SequentialCommandGroup(
      new WaitCommand(0.3),
      new ParallelRaceGroup(new DriveRamsete(driveTrain, "4BallPart1").robotRelative(), new ManualIndexer(shooter, indexer, acquisition)),
      new ShootAtDistance(indexer, shooter, acquisition, 2.432373),
      new DriveRamsete(driveTrain, "4BallPart2").robotRelative(),
      new ParallelRaceGroup(new DriveRamsete(driveTrain, "4BallPart3").robotRelative(), new ManualIndexer(shooter, indexer, acquisition)),
      new DriveRamsete(driveTrain, "4BallPart4").robotRelative(),
      new ShootAtDistance(indexer, shooter, acquisition, 5)
    );


    //Start Position: Anywhere on the field - Bumpers pushed against tarmac, facing team ball
    final SequentialCommandGroup twoBallAutonomous = new SequentialCommandGroup(
      new WaitCommand(0.3),
      new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomous").robotRelative(), new ManualIndexer(shooter, indexer, acquisition)),
      new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomousPart2").robotRelative(), new ManualIndexer(shooter, indexer, acquisition)),
      new ShootAtDistance(indexer, shooter, acquisition, 2),
      new DriveRamsete(driveTrain, "2BallAutonomousPart3")

    );
  }

   

}
