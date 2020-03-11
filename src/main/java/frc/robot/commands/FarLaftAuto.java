/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LimelightServo;
import frc.robot.subsystems.Staging;
import frc.robot.subsystems.TurretRotator;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FarLaftAuto extends SequentialCommandGroup {
  /**
   * Creates a new FarLaftAuto.
   */
  public FarLaftAuto(Drivetrain drivetrain, Intake intake, TurretRotator turretrotator, LimelightServo limelightservo, Launcher launcher, Hood hood, Staging staging) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(intake::intake, intake),
      new DriveDistance(4.5, drivetrain),
      new InstantCommand(intake::stop, intake),
      new DriveDistance(-2, drivetrain),
      new TurnAngle(45, drivetrain),
      new DriveDistance(-10, drivetrain),
      new AutoAimFinishes(limelightservo, turretrotator, hood, launcher)
    );
  }
}
