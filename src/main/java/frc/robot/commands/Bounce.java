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
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Bounce extends SequentialCommandGroup {
  /**
   * Creates a new Slalom.
   */
  public TrajectoryConfig config = Constants.trajectoryConfig;
  public Bounce(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    
   /* super(drivetrain,
      TrajectoryGenerator.generateTrajectory(
        List.of(
          drivetrain.getRobotPose(),
          //first circle
          new Pose2d(2.224, 3.502, Rotation2d.fromDegrees(95.7)),
          
          new Pose2d(2.248, 3.348, Rotation2d.fromDegrees(-76.3)),//remove once reversed
          new Pose2d(2.994, 1.482, Rotation2d.fromDegrees(-53.9))
          
          
          
        ),
      Constants.trajectoryConfig)
      
    );
    */
    super(new RamseteBase(drivetrain, TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(Constants.centerStartX,Constants.centerStartY, Rotation2d.fromDegrees(0)),
    new Pose2d(2.224, 3.502, Rotation2d.fromDegrees(95.7))),
       Constants.trajectoryConfig)),
       
    new RamseteBase(drivetrain, TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(2.224, 3.502, Rotation2d.fromDegrees(95.7)),

      new Pose2d(2.811, 1.715, Rotation2d.fromDegrees(124.4)),
      new Pose2d(3.743, 0.963, Rotation2d.fromDegrees(174.8)),
      new Pose2d(4.388, 1.697, Rotation2d.fromDegrees(-94.7)),
      new Pose2d(4.388, 3.685, Rotation2d.fromDegrees(-89.5)))
    , Constants.trajectoryConfigReverse)),

    new RamseteBase(drivetrain, TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(4.388, 3.685, Rotation2d.fromDegrees(-89.5)),
    
      new Pose2d(4.370, 1.952, Rotation2d.fromDegrees(-84.2)),
      new Pose2d(5.374, 1.087, Rotation2d.fromDegrees(0.6)),
      new Pose2d(6.473, 2.007, Rotation2d.fromDegrees(85.4)),
      new Pose2d(6.584, 3.842, Rotation2d.fromDegrees(88.4))
    ), Constants.trajectoryConfig)),
    new RamseteBase(drivetrain, TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(6.684, 3.692, Rotation2d.fromDegrees(88.4)),

      new Pose2d(7.422, 2.832, Rotation2d.fromDegrees(166.5)),
      new Pose2d(8.214, 2.636, Rotation2d.fromDegrees(165.9))
    ), Constants.trajectoryConfigReverse))
       );
  }
}
