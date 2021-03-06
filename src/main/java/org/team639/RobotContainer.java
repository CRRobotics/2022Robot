// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import java.util.TreeMap;

import org.team639.auto.*;
import org.team639.commands.AimbotLED;
import org.team639.commands.Acquisition.*;
import org.team639.commands.Climber.ToggleClimber;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final Climber climb = new Climber();
  private final LED candle = new LED();

  // Command Declaration
  AutonomousRoutines auton = new AutonomousRoutines();
  TeleopRoutines tele = new TeleopRoutines();

  // Drive
  private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain);
  private final ToggleGears shiftGears = new ToggleGears(driveTrain);
  private final ReverseHeading swap = new ReverseHeading(driveTrain);
  //public final TurnToBall turnToBall = new TurnToBall(driveTrain);

  // Acquisition
  private final ToggleAcquisition toggleAcquisition = new ToggleAcquisition(acquisition);

  // Indexer
  private final SpitCargo eject = new SpitCargo(shooter, indexer, acquisition);
  private final ManualIndexer index = new ManualIndexer(shooter, indexer, acquisition);

  // Shooter
  private final ShootClosedLoop fenderShot = new ShootClosedLoop(indexer, shooter, acquisition,
      Constants.ShooterConstants.fenderRPM, Constants.ShooterConstants.fenderAngle);

  
  private final ShootClosedLoop lowShot = new ShootClosedLoop(indexer, shooter, acquisition,
      2000, 0.2);
  private final ToggleActuator resetHood = new ToggleActuator(shooter);
  private final AimbotLED locked = new AimbotLED(candle);

  //Climber
  private final ToggleClimber toggleClimb = new ToggleClimber(climb);

  public static SendableChooser<DriveLayout> driveMode = new SendableChooser<>();
  public static SendableChooser<AutonMode> autoMode = new SendableChooser<>();
  public static SendableChooser<String> songChooser = new SendableChooser<>();
  public static SendableChooser<AllianceColor> allianceChooser = new SendableChooser<>();
  public static SendableChooser<LEDMode> ledChooser = new SendableChooser<>();

  public static final TrajectoryFactory factory = new TrajectoryFactory("paths");
  public static TreeMap<Double, AngleSpeed> shootMap = new TreeMap<>();

  static {
    shootMap.put(1.0, new AngleSpeed(.9,2900));
    shootMap.put(1.92, new AngleSpeed(.7, 3200));
    shootMap.put(2.77, new AngleSpeed(.65, 3400));
    shootMap.put(4.27, new AngleSpeed(.5, 3800));
    shootMap.put(5.46, new AngleSpeed(.4, 4150));
    shootMap.put(6.85, new AngleSpeed(.15, 4600));
  }

  static {
    driveMode.setDefaultOption("Arcade Standard", DriveLayout.Arcade);
    driveMode.addOption("Arcade InversedK", DriveLayout.ArcadeInverseK);
    driveMode.addOption("Curvature", DriveLayout.CurvatureDrive);
    driveMode.addOption("Tank", DriveLayout.Tank);

    SmartDashboard.putData("Drive Layout", driveMode);
  }

  static {
    autoMode.setDefaultOption("ShootAndMove", AutonMode.Shoot);
    autoMode.addOption("AutoCross Line", AutonMode.ForwardTest);
    autoMode.addOption("2BallAuto", AutonMode.TwoBall);
    autoMode.addOption("2BallAutoMidBall", AutonMode.TwoBallMidZone);
    autoMode.addOption("4BallAuto", AutonMode.FourBall);
    autoMode.addOption("BounceTest", AutonMode.BounceTest);
    SmartDashboard.putData("Auto Mode", autoMode);
  }

  static {
    songChooser.setDefaultOption("Old Town Road", Constants.DJConstants.old_town_road);
    songChooser.addOption("Industry Baby", Constants.DJConstants.industry_baby);
    SmartDashboard.putData("SongChooser", songChooser);

  }

  static {
    allianceChooser.setDefaultOption("RED", AllianceColor.red);
    allianceChooser.addOption("BLUE", AllianceColor.blue);
    SmartDashboard.putData("Alliance", allianceChooser);
  }

  static {
    ledChooser.setDefaultOption("Aimbot", LEDMode.aimbot);
    ledChooser.addOption("LEDSwapCade", LEDMode.swapcade);
    ledChooser.addOption("Fire", LEDMode.fire);
    ledChooser.addOption("GamerMode", LEDMode.gamerMode);
    SmartDashboard.putData(ledChooser);
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
   * 
   * @return Song to be chosen
   */
  public static String songChooser() {
    return songChooser.getSelected();
  }

  /**
   * Returns the chosen alliance color
   */
  public static AllianceColor getAllianceColor() {
    return allianceChooser.getSelected();
  }

  /**
   * Returns the chosen LEDMode
   * 
   * @return chosen LED configuration
   */
  public static LEDMode getLedMode() {
    return ledChooser.getSelected();
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
    // Driver
    ControllerWrapper.DriverRightBumper.whenPressed(shiftGears);
    ControllerWrapper.DriverLeftBumper.whenPressed(swap);
    ControllerWrapper.DriverButtonA.whenHeld(new DJRobot(driveTrain, 10));
    ControllerWrapper.DriverButtonX.whenPressed(new TurnToAngleRelative(driveTrain).withTimeout(1));
    ControllerWrapper.DriverButtonY.whenPressed(tele.aimbotshot);
    //ControllerWrapper.DriverButtonB.whenPressed(turnToBall.withTimeout(1));

    // Controller
    
    
    ControllerWrapper.ControlRightBumper.whenHeld(index);
    ControllerWrapper.ControlLeftBumper.whenHeld(eject);

    ControllerWrapper.ControlButtonY.whenPressed(fenderShot);
    ControllerWrapper.ControlButtonB.whenPressed(toggleAcquisition);
    ControllerWrapper.ControlButtonX.whenPressed(tele.aimbotshot);
    ControllerWrapper.ControlButtonA.whenPressed(toggleClimb);
    ControllerWrapper.ControlDPadRight.whenPressed(lowShot);
    ControllerWrapper.ControlDPadLeft.whenPressed(new AutoShootAtDistance(indexer, shooter, acquisition, candle));
    ControllerWrapper.ControlDPadDown.whenPressed(new ShootAtDistance(indexer, shooter, acquisition, candle, 2.5));

    // RESET BUTTONS
    ControllerWrapper.ControlDPadUp.whenPressed(resetHood);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command auto;
    switch (getAutonomousMode()) {
      default:
        auto = auton.ShootMove;
        break;
      case ForwardTest:
        auto = auton.forwardTest;
        break;
      case TwoBall:
        auto = auton.twoBallAutonomous;
        break;
      case TwoBallMidZone:
        auto = auton.twoBallAutonomousZone2;
        break;
      case Shoot:
        auto = auton.ShootMove;
        break;
      case FourBall:
        auto = auton.fourBall_v2_optimized;
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
    CommandScheduler.getInstance().setDefaultCommand(candle, locked);
  }

  class TeleopRoutines {
    final ParallelCommandGroup aimbotshot = new ParallelCommandGroup(new TurnToAngleRelative(driveTrain).withTimeout(1), new AutoShootAtDistance(indexer, shooter, acquisition, candle));
  }

  class AutonomousRoutines {
    // Testing commands
    final SequentialCommandGroup forwardTest = new SequentialCommandGroup(
        new DriveRamsete(driveTrain, "2BallAutonomousPart3"));
    final SequentialCommandGroup bounceTest = new SequentialCommandGroup(
        new DriveRamsete(driveTrain, "bounce1"),
        new DriveRamsete(driveTrain, "bounce2"),
        new DriveRamsete(driveTrain, "bounce1"),
        new DriveRamsete(driveTrain, "bounce3"));
    
    final SequentialCommandGroup ShootMove = new SequentialCommandGroup(
      new ShootAtDistance(indexer, shooter, acquisition, candle, 1.65),
      new DriveRamsete(driveTrain, "2BallAutonomous")
      );

    // Start Position: 7.635, 1.786 - Facing bottom team ball, bumpers against the
    // tarmac edge
    // Unfortunately, we will not have Rohits jumper
    final SequentialCommandGroup fourBallAutonomousAndRohitsJumper = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "4BallPart1").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ShootAtDistance(indexer, shooter, acquisition, candle, 2.432373),
        new DriveRamsete(driveTrain, "4BallPart2").robotRelative(),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "4BallPart3").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new DriveRamsete(driveTrain, "4BallPart4").robotRelative(),
        new ShootAtDistance(indexer, shooter, acquisition, candle, 5));

    // Start Position: 6.44, 2.62 - Side of bumpers lined up against the tarmac,
    // with the front edge facing touching the front tarmac
    final SequentialCommandGroup fourBall_v2 = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "4Ball_v2Part1"),
            new ShootAtDistanceTimed(indexer, shooter, acquisition, 3.5, 1700)),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "4Ball_v2Part2"),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ManualIndexer(shooter, indexer, acquisition).withTimeout(0.75),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "4Ball_v2Part3"),
            new ManualIndexer(shooter, indexer, acquisition).withTimeout(.2)),
        new ShootAtDistance(indexer, shooter, acquisition, candle, 3.5));

    final SequentialCommandGroup fourBall_v2_optimized = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "4BALLTRUE1"),
            new ShootAtDistanceTimed(indexer, shooter, acquisition, 3.85, 1700)),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "4BALLTRUE2"),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ManualIndexer(shooter, indexer, acquisition).withTimeout(1.2),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "4BALLTRUE3"),
            new ManualIndexer(shooter, indexer, acquisition).withTimeout(.5)),
        //new ParallelCommandGroup(new TurnToAngleRelative(driveTrain).withTimeout(1),new ShootAtDistance(indexer, shooter, acquisition, candle, 3.5)));
        new ParallelCommandGroup(new TurnToAngleRelative(driveTrain).withTimeout(1), new AutoShootAtDistance(indexer, shooter, acquisition, candle)));

    // Coding gods, take the wheel
    final SequentialCommandGroup fiveBall = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "4Ball_v2Part1"),
            new ShootAtDistanceTimed(indexer, shooter, acquisition, 3.5, 1700)),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "4Ball_v2Part2"),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ManualIndexer(shooter, indexer, acquisition).withTimeout(0.75),
        new ParallelCommandGroup(new DriveRamsete(driveTrain, "5BallPart3"),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ParallelCommandGroup(new ShootAtDistanceTimed(indexer, shooter, acquisition, 3.5, 5),
            new SequentialCommandGroup(new WaitCommand(1.6), new DriveRamsete(driveTrain, "5BallPart4"))));

    // Start Position: Anywhere on the field - Bumpers pushed against tarmac, facing
    // team ball
    final SequentialCommandGroup twoBallAutonomous = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomous").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomousPart2").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new TurnToAngleRelative(driveTrain).withTimeout(1),
        new ShootAtDistance(indexer, shooter, acquisition, candle, 1.65));

    // Start Position: Middle ball of 2 ball 2wsZautonomous - Bumpers pushed against tarmac, facing
    // team ball
    final SequentialCommandGroup twoBallAutonomousZone2 = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomous").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ParallelRaceGroup(new DriveRamsete(driveTrain, "2BallAutonomousPart2").robotRelative(),
            new ManualIndexer(shooter, indexer, acquisition)),
        new ShootAtDistance(indexer, shooter, acquisition, candle, 1.65),
        new DriveRamsete(driveTrain, "2BallAutonomousPart3"));

  }

}
