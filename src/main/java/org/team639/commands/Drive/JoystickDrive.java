// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.RobotContainer;

import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Controls the robots movements
 */
public class JoystickDrive extends CommandBase {
  private DriveTrain driveTrain;
  private double mQuickStopAccumulator;


  /** Creates a new JoystickDrive. */
  public JoystickDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Joystick drive initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotContainer.getDriveLayout()) {
      case Swapcade:
        if(!driveTrain.isReversedHeading())
          arcadeDrive(handleDeadband(ControllerWrapper.DriverController.getLeftY(), Constants.ControlboardConstants.kJoystickThreshold), handleDeadband(ControllerWrapper.DriverController.getRightX(), Constants.ControlboardConstants.kJoystickThreshold));
        else
          arcadeReversed(handleDeadband(ControllerWrapper.DriverController.getLeftY(), Constants.ControlboardConstants.kJoystickThreshold), handleDeadband(ControllerWrapper.DriverController.getRightX(), Constants.ControlboardConstants.kJoystickThreshold));
        break;
      case Arcade:
        arcadeDrive(handleDeadband(ControllerWrapper.DriverController.getLeftY(), Constants.ControlboardConstants.kJoystickThreshold), handleDeadband(ControllerWrapper.DriverController.getRightX(), Constants.ControlboardConstants.kJoystickThreshold));
        break;
      case ArcadeReversed:
        arcadeReversed(handleDeadband(ControllerWrapper.DriverController.getLeftY(), Constants.ControlboardConstants.kJoystickThreshold), handleDeadband(ControllerWrapper.DriverController.getRightX(), Constants.ControlboardConstants.kJoystickThreshold));
      case CheesyDrive:
        cheezyDrive(ControllerWrapper.DriverController.getLeftY(), ControllerWrapper.DriverController.getRightX(), false);
        break;
      case Tank:
        tankDrive(ControllerWrapper.DriverController.getLeftY(), ControllerWrapper.DriverController.getRightY());
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Joystick drive peelin' out");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Arcade drive given a speed and turning magnitude
   * 
   * @param speed     Speed in percent
   * @param turnValue Magnitude of turning
   */
  public void arcadeDrive(double speed, double turnValue) {
    speed *= -Constants.DriveConstants.driveMultiplier;

    double turnMultiplier = 1 - speed;
    if (turnMultiplier < 1d / 3d)
      turnMultiplier = 1d / 3d;
    if (turnMultiplier > 2d / 3d)
      turnMultiplier = 2d / 3d;
    turnValue = turnValue * turnMultiplier;
 
    double left = speed + turnValue;
    double right = speed - turnValue;

    driveTrain.setSpeedsPercent(left, right);
  }

  /**
   * Arcade drive given a speed and turning magnitude
   * This is the reversed version
   * @param speed     Speed in percent
   * @param turnValue Magnitude of turning
   */
  public void arcadeReversed(double speed, double turnValue)
  {
    speed *= -Constants.DriveConstants.driveMultiplier;

    double turnMultiplier = 1 - speed;
    if (turnMultiplier < 1d / 3d)
      turnMultiplier = 1d / 3d;
    if (turnMultiplier > 2d / 3d)
      turnMultiplier = 2d / 3d;
    turnValue = turnValue * turnMultiplier;
 
    double right = speed + turnValue;
    double left = speed - turnValue;

    driveTrain.setSpeedsPercent(-left, -right);
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    leftSpeed *= Constants.DriveConstants.driveMultiplier;
    rightSpeed *= 
    Constants.DriveConstants.driveMultiplier;
    driveTrain.setSpeedsPercent(leftSpeed, rightSpeed);
  }

  /**
   * Implementation of FRC 254's 'Cheezy Drive'
   * @param throttle Magnitude of throttle
   * @param wheel Magnitude of turning
   * @param isQuickTurn Override in order to turn in place or at slow speeds
   */
  public void cheezyDrive(double throttle, double wheel, boolean isQuickTurn) {
    wheel = -handleDeadband(wheel, Constants.DriveConstants.kWheelDeadband);
    throttle = handleDeadband(throttle, Constants.DriveConstants.kThrottleDeadband);

    double overPower;
    double angularPower;

    if (this.quickTurnOverride(throttle)) {
      if (Math.abs(throttle) < 0.2) {
        double alpha = 0.1;
        mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * limit(wheel, 1.0) * 2;
      }
      overPower = 1.0;
      angularPower = wheel;
    } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * Constants.DriveConstants.kTurnSensitivity - mQuickStopAccumulator;
      if (mQuickStopAccumulator > 1) {
        mQuickStopAccumulator -= 1;
      } else if (mQuickStopAccumulator < -1) {
        mQuickStopAccumulator += 1;
      } else {
        mQuickStopAccumulator = 0.0;
      }
    }

    double rightPwm = throttle + angularPower;
    double leftPwm = throttle - angularPower;
    if (leftPwm > 1.0) {
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }
    driveTrain.setSpeedsPercent(leftPwm * Constants.DriveConstants.driveMultiplier, rightPwm * Constants.DriveConstants.driveMultiplier);
  }

  public void cheezyReversed(double throttle, double wheel, boolean isQuickTurn)
  {
    wheel = -handleDeadband(wheel, Constants.DriveConstants.kWheelDeadband);
    throttle = handleDeadband(throttle, Constants.DriveConstants.kThrottleDeadband);

    double overPower;
    double angularPower;

    if (this.quickTurnOverride(throttle)) {
      if (Math.abs(throttle) < 0.2) {
        double alpha = 0.1;
        mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * limit(wheel, 1.0) * 2;
      }
      overPower = 1.0;
      angularPower = wheel;
    } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * Constants.DriveConstants.kTurnSensitivity - mQuickStopAccumulator;
      if (mQuickStopAccumulator > 1) {
        mQuickStopAccumulator -= 1;
      } else if (mQuickStopAccumulator < -1) {
        mQuickStopAccumulator += 1;
      } else {
        mQuickStopAccumulator = 0.0;
      }
    }

    double leftPwm = throttle - angularPower;
    double rightPwm = throttle + angularPower;
    if (leftPwm > 1.0) {
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }
    driveTrain.setSpeedsPercent(-leftPwm * Constants.DriveConstants.driveMultiplier, -rightPwm * Constants.DriveConstants.driveMultiplier);
  }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public boolean quickTurnOverride(double throttle) {
    if (throttle < Constants.DriveConstants.overrideThreshhold)
      return true;
    return false;
  }

  /**
   * Limits the given input to the given magnitude.
   */
  public static double limit(double v, double limit) {
    return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
  }
}
