// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.commands.Drive;

import org.team639.RobotContainer;

import org.team639.controlboard.ControllerWrapper;
import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.MathUtil;
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
    double yValue = handleDeadband(ControllerWrapper.DriverController.getLeftY(),
        Constants.ControlboardConstants.kJoystickThreshold);
    double xValue = handleDeadband(ControllerWrapper.DriverController.getRightX(),
        Constants.ControlboardConstants.kJoystickThreshold);

    switch (RobotContainer.getDriveLayout()) {
      case Arcade:
        if (!driveTrain.isReversedHeading())
          arcadeDrive(
              yValue,
              xValue);
        else
          arcadeDrive(
              -yValue,
              xValue);
        break;
      case ArcadeInverseK:
        if (!driveTrain.isReversedHeading())
          arcadeInverseK(yValue, xValue, false);
        else
          arcadeInverseKRev(
              yValue,
              xValue, false);
        break;
      case CurvatureDrive:
        if (!driveTrain.isReversedHeading())
          cheezyDrive(yValue, xValue, quickTurnOverride(yValue));
        else
          cheezyDrive(-yValue, xValue, quickTurnOverride(yValue));
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
    speed *= Constants.DriveConstants.driveMultiplier;
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

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftSpeed *= Constants.DriveConstants.driveMultiplier;
    rightSpeed *= Constants.DriveConstants.driveMultiplier;
    driveTrain.setSpeedsPercent(leftSpeed, rightSpeed);
  }

  /**
   * Arcade drive using inversed kinematics
   * 
   * @param speed        Magnitude of speed
   * @param turning      Magnitude of turning
   * @param squareInputs Determines whether to square controller input
   */
  public void arcadeInverseK(double speed, double turning, boolean squareInputs) {
    // speed = MathUtil.clamp(speed, -1.0, 1.0);
    // turning = MathUtil.clamp(turning, -1.0, 1.0);

    if (squareInputs) {
      speed = Math.copySign(speed * speed, speed);
      turning = Math.copySign(turning * turning, turning);
    }

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(turning)), speed);

    // Case Forward translational movement
    if (Math.signum(speed) >= 0) {
      if (Math.signum(turning) >= 0) {
        leftSpeed = maxInput;
        rightSpeed = speed - turning;
      } else {
        leftSpeed = speed + turning;
        rightSpeed = maxInput;
      }

      // Case Backwards translational movement
    } else {
      if (Math.signum(turning) >= 0) {
        leftSpeed = speed + turning;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = speed - turning;
      }
    }

    leftSpeed = MathUtil.clamp(leftSpeed, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);

    driveTrain.setSpeedsPercent(leftSpeed * Constants.DriveConstants.driveMultiplier,
        rightSpeed * Constants.DriveConstants.driveMultiplier);
  }

    /**
   * Arcade drive using inversed kinematics reversed
   * 
   * @param speed        Magnitude of speed
   * @param turning      Magnitude of turning
   * @param squareInputs Determines whether to square controller input
   */
  public void arcadeInverseKRev(double speed, double turning, boolean squareInputs) {
    // speed = MathUtil.clamp(speed, -1.0, 1.0);
    // turning = MathUtil.clamp(turning, -1.0, 1.0);

    if (squareInputs) {
      speed = Math.copySign(speed * speed, speed);
      turning = Math.copySign(turning * turning, turning);
    }

    double leftSpeed;
    double rightSpeed;

    double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(turning)), speed);

    // Case Forward translational movement
    if (Math.signum(speed) >= 0) {
      if (Math.signum(turning) >= 0) {
        leftSpeed = maxInput;
        rightSpeed = speed - turning;
      } else {
        leftSpeed = speed + turning;
        rightSpeed = maxInput;
      }

      // Case Backwards translational movement
    } else {
      if (Math.signum(turning) >= 0) {
        leftSpeed = speed + turning;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = speed - turning;
      }
    }

    leftSpeed = MathUtil.clamp(leftSpeed, -1.0, 1.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);

    driveTrain.setSpeedsPercent(-rightSpeed * Constants.DriveConstants.driveMultiplier,
        -leftSpeed * Constants.DriveConstants.driveMultiplier);
  }
  /**
   * Implementation of FRC 254's 'Cheezy Drive'
   * 
   * @param throttle    Magnitude of throttle
   * @param wheel       Magnitude of turning
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
    driveTrain.setSpeedsPercent(leftPwm * Constants.DriveConstants.driveMultiplier,
        rightPwm * Constants.DriveConstants.driveMultiplier);
  }

  public void cheezyReversed(double throttle, double wheel, boolean isQuickTurn) {
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
    driveTrain.setSpeedsPercent(-leftPwm * Constants.DriveConstants.driveMultiplier,
        -rightPwm * Constants.DriveConstants.driveMultiplier);
  }

  // /**
  //  * Implementation of WPILibs implementation of curvature drive which in turn is
  //  * an implementation of CheezyDrive
  //  */
  // public void curvatureDrive(double speed, double turning, boolean quickTurn) {
  //   speed = MathUtil.clamp(speed, -1.0, 1.0);
  //   turning = MathUtil.clamp(turning, -1.0, 1.0);

  //   double leftSpeed;
  //   double rightSpeed;

  //   if (quickTurn) {
  //     leftSpeed = speed + turning;
  //     rightSpeed = speed - turning;
  //   } else {
  //     leftSpeed = speed + Math.abs(speed) * turning;
  //     rightSpeed = speed - Math.abs(speed) * turning;
  //   driveTrain.setSpeedsPercent(leftSpeed, rightSpeed);
  //   }

  //   // Normalize wheel speeds
  //   double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
  //   if (maxMagnitude > 1.0) {
  //     leftSpeed /= maxMagnitude;
  //     rightSpeed /= maxMagnitude;
  //   }
  // }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public boolean quickTurnOverride(double throttle) {
    if (Math.abs(throttle) < Constants.DriveConstants.overrideThreshhold)
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
