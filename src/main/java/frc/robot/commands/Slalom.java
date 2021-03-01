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
          new Pose2d(Constants.edgeStartX, Constants.edgeStartY,Rotation2d.fromDegrees(0)),
          //to the circle
         new Pose2d(2.365, 1.621, Rotation2d.fromDegrees(60.9)),
         new Pose2d(4.485, 3.289, Rotation2d.fromDegrees(-0.9)),
         new Pose2d(6.445, 1.972, Rotation2d.fromDegrees(-54.0)),
          // //around the circle
          new Pose2d(7.854,0.900, Rotation2d.fromDegrees(33.5)),
          new Pose2d(7.803,2.231, Rotation2d.fromDegrees(166.9)),
          new Pose2d(6.705,1.341, Rotation2d.fromDegrees(-126.8)),
          //to end
          new Pose2d(5.181, 0.634, Rotation2d.fromDegrees(-178.4)),
          new Pose2d(2.553, 1.074, Rotation2d.fromDegrees(143.5)),
          new Pose2d(1.719, 2.207, Rotation2d.fromDegrees(126.4))
          //  new Pose2d(6.891, 1.060, Rotation2d.fromDegrees(-58.5)),
          // // new Pose2d(7.218, 0.716, Rotation2d.fromDegrees(-29.5)),
          //  new Pose2d(7.638, 0.611, Rotation2d.fromDegrees(0.6)),
          // // new Pose2d(8.110, 0.873, Rotation2d.fromDegrees(53.1)),
          //  new Pose2d(8.301, 1.334, Rotation2d.fromDegrees(82.1)),
          // // new Pose2d(8.280, 1.689, Rotation2d.fromDegrees(105.9)),
          //  new Pose2d(8.062, 2.048, Rotation2d.fromDegrees(135.4)),
          // // new Pose2d(7.672, 2.259, Rotation2d.fromDegrees(166.6)),
          //  new Pose2d(7.299, 2.256, Rotation2d.fromDegrees(-166.7)),
          // // new Pose2d(6.848, 1.957, Rotation2d.fromDegrees(-124.6)),
          //  new Pose2d(6.714, 1.614, Rotation2d.fromDegrees(-132.3)),
          // //to the end
          //  //new Pose2d(6.516, 1.038, Rotation2d.fromDegrees(-142.5)),
          //  new Pose2d(6.018, 0.737, Rotation2d.fromDegrees(-169.6)),
          // //$$new Pose2d(5.133, 0.791, Rotation2d.fromDegrees(-177.6)),
          // //new Pose2d(4.234, 0.730, Rotation2d.fromDegrees(-175.6)),
          // new Pose2d(3.577, 0.674, Rotation2d.fromDegrees(-174.9)),
          // new Pose2d(3.044, 0.661, Rotation2d.fromDegrees(174.4)),
          // //$$new Pose2d(2.644, 0.778, Rotation2d.fromDegrees(149.6)),
          // new Pose2d(2.232, 1.203, Rotation2d.fromDegrees(119.5)),
          // //new Pose2d(1.852, 1.960, Rotation2d.fromDegrees(131.4)),
          // new Pose2d(1.057, 2.123, Rotation2d.fromDegrees(-177.4))

        ),
      Constants.trajectoryConfig)
    );
  }
}
