// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639;

import org.team639.lib.Constants;
import org.team639.subsystems.JeVoisInterface;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  Compressor phCompressor = new Compressor(Constants.Ports.PneumaticsModuleType.phCompressorID, PneumaticsModuleType.REVPH);

  public static double horizontalDistanceToTarget;
  public static double horizontalAngleToTarget;

  public static double runningHorizontalAngle;
  public static double runningHorizontalDistance;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //phCompressor.enableAnalog(0, Constants.maxCompressor);
    m_robotContainer = new RobotContainer();



  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    SmartDashboard.putNumber("Current Pressure", getCompressorPressure());

    System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("HorizontalDistance").getDouble(-1.0));
    System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("HorizontalAngle").getDouble(361.0));
    horizontalAngleToTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("HorizontalAngle").getDouble(-1.0);
    horizontalDistanceToTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("HorizontalDistance").getDouble(361.0);
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getAutonomousCommand().schedule();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // JeVoisInterface test = new JeVoisInterface(false);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public double getCompressorPressure()
  {
    return phCompressor.getPressure();
  }

  /**
   * Returns the running horizontal angle. This makes sure that the last value is not undefined
   * @return runningHorizontalAngle of horizontal angle
   */
  public static double getAngleToTarget()
  {
    if(horizontalAngleToTarget != 361.0)
    {
      runningHorizontalAngle = horizontalAngleToTarget;
    }
    return runningHorizontalAngle;
  }

  /**
   * Returns the running horizontal distance. This makes sure that the last value is not undefined
   * @return runningHorizontalDistance of horizontal distance
   */
  public static double getDistanceToTarget()
  {
    if(horizontalDistanceToTarget != -1.0)
    {
      runningHorizontalDistance = horizontalDistanceToTarget;
    }
    return runningHorizontalDistance;
  }
}
