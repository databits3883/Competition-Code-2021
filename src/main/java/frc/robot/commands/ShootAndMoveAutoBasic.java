/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Staging;
import frc.robot.subsystems.TurretRotator;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndMoveAutoBasic extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndMoveAutoBasic.
   */
  public ShootAndMoveAutoBasic(TurretHood m_hood, TurretRotator m_turretRotator, Launcher m_launcher, Staging m_staging, Drivetrain m_drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new FullTurretAim(22.5, 98, -78.7, m_hood, m_turretRotator, m_launcher),
      new ShootThreePowerCells(m_staging), 
      new DriveDistance(1, m_drivetrain)
    );
  }
}
