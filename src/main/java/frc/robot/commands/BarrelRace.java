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

public class BarrelRace extends RamseteBase {
  /**
   * Creates a new Slalom.
   */
  public BarrelRace(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain,
      TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(Constants.centerStartX, Constants.centerStartY, Rotation2d.fromDegrees(0)),
          //first circle
          new Pose2d(4.179, 2.069, Rotation2d.fromDegrees(-43.9)),
          new Pose2d(4.057, 0.727, Rotation2d.fromDegrees(-165.4)),
          new Pose2d(2.849, 1.310, Rotation2d.fromDegrees(109.1)),
          new Pose2d(3.811, 2.879, Rotation2d.fromDegrees(-2.5)),
          //second circle
         new Pose2d(6.605, 2.725, Rotation2d.fromDegrees(62.8)),
         new Pose2d(5.475, 3.384, Rotation2d.fromDegrees(-125.0)),
         new Pose2d(5.710, 1.446, Rotation2d.fromDegrees(-48.5)),
        //  //third circle 
        new Pose2d(6.697, 0.875, Rotation2d.fromDegrees(-11.4)),
        new Pose2d(8.002, 1.190, Rotation2d.fromDegrees(59.0)),
        new Pose2d(7.819, 2.000, Rotation2d.fromDegrees(158.1)),
         new Pose2d(6.270, 2.030, Rotation2d.fromDegrees(-178.8)),
        //to the end
        new Pose2d(4.073, 2.196, Rotation2d.fromDegrees(173.8)),
        new Pose2d(0.648, 1.993, Rotation2d.fromDegrees(-177.4))
        ),
      Constants.trajectoryConfig)
    );
  }
}
