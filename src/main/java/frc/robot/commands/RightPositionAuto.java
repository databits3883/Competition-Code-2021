/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomStagingBelt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LimelightServo;
import frc.robot.subsystems.TurretRotator;
import frc.robot.subsystems.UpperStagingBelt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RightPositionAuto extends SequentialCommandGroup {
  /**
   * Creates a new RightPositionAuto.
   */
  public RightPositionAuto(Drivetrain drivetrain, Intake intake, TurretRotator turretrotator, LimelightServo limelightservo, Launcher launcher, Hood hood, UpperStagingBelt upperStagingBelt, BottomStagingBelt bottomStagingBelt) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      
      new FullTurretAim(39.32, 120, -75, hood, turretrotator, launcher),
      new ShootThreePowerCells(upperStagingBelt, bottomStagingBelt).withTimeout(5),
      new InstantCommand(intake::intake, intake),
      new DriveDistance(13, drivetrain),
      
      new FullTurretAim(41, 110.495, -110, hood, turretrotator, launcher),
      new ShootThreePowerCells(upperStagingBelt, bottomStagingBelt)
        
      


    );
  }
}
