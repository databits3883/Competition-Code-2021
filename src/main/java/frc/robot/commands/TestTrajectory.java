/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TestTrajectory extends RamseteBase {
  /**
   * Creates a new TestTrajectory.
   */
  public TestTrajectory(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain, 
    TrajectoryGenerator.generateTrajectory(drivetrain.getRobotPose(), new ArrayList<Translation2d>(), new Pose2d(0,0,new Rotation2d(0)), Constants.trajectoryConfig));
  }
}
