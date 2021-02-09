/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Slalom extends RamseteBase {
  /**
   * Creates a new Slalom.
   */
  public Slalom(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain,
      TrajectoryGenerator.generateTrajectory(
        List.of(
          drivetrain.getRobotPose(),
          new Pose2d(1.50,0.737,Rotation2d.fromDegrees(30.9)),
          new Pose2d(2.23,1.55,Rotation2d.fromDegrees(54.7)),
          new Pose2d(3.03,2.31,Rotation2d.fromDegrees(28.3)),
          new Pose2d(4.48,2.36,Rotation2d.fromDegrees(3.5)),
          new Pose2d(6.01,2.17,Rotation2d.fromDegrees(-26.5)),
          new Pose2d(6.67,1.51,Rotation2d.fromDegrees(-58.3)),
          new Pose2d(7.28,1.04,Rotation2d.fromDegrees(-10.4)),
          new Pose2d(8.04,1.40,Rotation2d.fromDegrees(60)),
          new Pose2d(7.49,2.26,Rotation2d.fromDegrees(176.3)),
          new Pose2d(6.71,1.85,Rotation2d.fromDegrees(-135.5)),
          new Pose2d(6.05,1.05,Rotation2d.fromDegrees(-141.5)),
          new Pose2d(4.63,0.74,Rotation2d.fromDegrees(-174.9)),
          new Pose2d(2.61,0.84,Rotation2d.fromDegrees(162.7)),
          new Pose2d(2.04,1.57,Rotation2d.fromDegrees(123.3)),
          new Pose2d(1.20,2.14,Rotation2d.fromDegrees(175.9)),
          new Pose2d(0.88,2.11,Rotation2d.fromDegrees(-173.1))
          

        ),
      Constants.trajectoryConfig)
    );
  }
}
