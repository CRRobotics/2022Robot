// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.auto;

import org.team639.RobotContainer;
import org.team639.lib.Constants;
import org.team639.subsystems.DriveTrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DriveRamsete extends RamseteCommand {

  private Trajectory trajectory;
  private DriveTrain driveTrain;
  private boolean resetPosition;

  /** Creates a new DriveRamsete. */
  public DriveRamsete(DriveTrain driveTrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(trajectory, driveTrain::getPose, new RamseteController(), Constants.AutoConstants.feedforward,
        Constants.AutoConstants.kinematics, driveTrain::getWheelSpeeds, driveTrain.getLeftPIDController(),
        driveTrain.getRightPIDController(), driveTrain::setVoltages, driveTrain);

    this.resetPosition = true;
    this.trajectory = trajectory;
    this.driveTrain = driveTrain;
  }

  public DriveRamsete(DriveTrain drivetrain, String name) {
    this(drivetrain, RobotContainer.factory.getTrajectory(name));
  }

  // Resets the drivetrain to the begining of the trajectory
  public DriveRamsete robotRelative() {
    this.resetPosition = true;
    return this;
  }

  // Make the trajectory relative to the field
  public DriveRamsete fieldRelative() {
    this.resetPosition = false;
    return this;
  }

  @Override
  public void initialize() {
    super.initialize();

    // drivetrain.toggleGearHigh();

    if (resetPosition) {
      driveTrain.resetOdometry(trajectory.getInitialPose());
    }
    System.out.println("Drive RAMSETE Has Been Initialized");
  }

  public void end()
  {
    driveTrain.setVoltages(0, 0);
    System.out.println("Drive RAMSETE Has Been Ended");
  }
}
