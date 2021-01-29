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
        drivetrain.getRobotPose(),
        List.of(
          new Translation2d(1.7,0.93),
          new Translation2d(2.67,2.13),
          new Translation2d(7.09,0.93),
          new Translation2d(8.29,1.12),
          new Translation2d(8.07,2.05),
          new Translation2d(8.07-0.78,1.85),
          new Translation2d(6.37,0.84),
          new Translation2d(2.77,0.84),
          new Translation2d(1.92,2.23)

        ),
        new Pose2d(1.12,2.53,Rotation2d.fromDegrees(180)),
      Constants.trajectoryConfig)
    );
  }
}
