/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightServo;
import frc.robot.subsystems.TurretRotator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LeftPositionAuto extends SequentialCommandGroup {
  /**
   * Creates a new LeftPositionAuto.
   */
  public LeftPositionAuto(Drivetrain drivetrain, Intake intake, TurretRotator turretrotator, LimelightServo limelightservo) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new DriveDistance(3.75, drivetrain),
      new TurnAngle(-90, drivetrain),
      new DriveDistance(10.66, drivetrain),
      
      new TurnAngle(90, drivetrain),
      new DriveDistance(13.33, drivetrain),
      new BallFollowing(drivetrain, turretrotator, limelightservo, intake)
      

      



    );
  }
}
