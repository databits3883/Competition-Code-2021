/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RamseteBase extends RamseteCommand {
  /**
   * Creates Ramsete controller command
   * @param drivetrain the drivetrain to require
   * @param trajectory the trajectory to follow
   */
  public RamseteBase(Drivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(trajectory, drivetrain::getRobotPose, new RamseteController(2.0, 0.7), Constants.robotKinematics, drivetrain::absoluteTankDriveMeters, drivetrain);
  }
}
