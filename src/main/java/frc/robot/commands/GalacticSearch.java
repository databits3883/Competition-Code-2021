// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretCameraAim;
import frc.robot.subsystems.TurretRotator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  /** Creates a new GalacticSearch. */
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  public GalacticSearch(Drivetrain drivetrain, TurretRotator turretRotator, TurretCameraAim limelightservo, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //initial position
      parallel(
        sequence(new ExtendIntake(intake), 
          new InstantCommand(intake::intake,intake)
        ),
        new InstantCommand(()-> drivetrain.setPose(1.100, 2.727, 0),drivetrain),
        new InstantCommand(()-> limelightservo.setPosition(139), limelightservo),
        new SetTurretAngle(Constants.maxTurretAngle, turretRotator)
      ),
      //determine and run path
      new SelectCommand(
        Map.of(
          path.kblueA, new RamseteBase(drivetrain, path.kblueA.trajectory),
          path.kblueB, new RamseteBase(drivetrain, path.kblueB.trajectory),
          path.kredA, new RamseteBase(drivetrain, path.kredA.trajectory),
          path.kredB, new RamseteBase(drivetrain, path.kredB.trajectory)), 
        this::determinePath),
      //stop the intake for convenience
      new InstantCommand(intake::stop,intake)
    );

  }
  public path determinePath(){
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    if (ty>=Constants.tySearchThreshold){
      if(tx>= Constants.txBlueSearchThreshold){
        return path.kblueA;
      }else{
        return path.kblueB;
      }
    }else{
      if(tx>= Constants.txRedSearchThreshold){
        return path.kredA;
      }else{
        return path.kredB;
      }
    }
  }

  enum path{
    kredA(List.of(
      new Pose2d(1.100,2.727, Rotation2d.fromDegrees(0)),
      new Pose2d(2.162,2.423,Rotation2d.fromDegrees(-24.4)),
      new Pose2d(3.701,1.583,Rotation2d.fromDegrees(-28.6)),
      new Pose2d(4.358,3.233,Rotation2d.fromDegrees(95.2)),
      new Pose2d(8.912,2.574,Rotation2d.fromDegrees(-4.1))

    )), 
    kredB(List.of(
      new Pose2d(1.100,2.727, Rotation2d.fromDegrees(0)),
      new Pose2d(1.961,2.747,Rotation2d.fromDegrees(35)),
      new Pose2d(3.656,1.698,Rotation2d.fromDegrees(-56.4)),
      new Pose2d(4.667,2.366,Rotation2d.fromDegrees(58.8)),
      new Pose2d(8.815,3.614,Rotation2d.fromDegrees(-15.8))
    )),
    kblueA(List.of(
      new Pose2d(1.100,2.727, Rotation2d.fromDegrees(0)),
      new Pose2d(4.332,0.965,Rotation2d.fromDegrees(-39.9)),
      new Pose2d(5.288,2.536,Rotation2d.fromDegrees(108.1)),
      new Pose2d(6.491,2.808,Rotation2d.fromDegrees(-57.9)),
      new Pose2d(8.904,1.547,Rotation2d.fromDegrees(9.8))
    )), 
    kblueB(List.of(
      new Pose2d(1.100,2.727, Rotation2d.fromDegrees(0)),
      new Pose2d(4.240,1.684,Rotation2d.fromDegrees(-31.5)),
      new Pose2d(5.701,2.594,Rotation2d.fromDegrees(78.3)),
      new Pose2d(7.355,1.980,Rotation2d.fromDegrees(-83.1)),
      new Pose2d(8.934,0.937,Rotation2d.fromDegrees(7.5))
    ));

    Trajectory trajectory;
    path(List<Pose2d> points ){
      this.trajectory = TrajectoryGenerator.generateTrajectory(points, Constants.slowTrajectorConfig);
    }
  }

}
