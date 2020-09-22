/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.TurretRotator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FullTurretAim extends ParallelCommandGroup {
  /**
   * Creates a new FullTurretAim.
   */
  public FullTurretAim(double hoodAngle, double turretAngle, double launchSpeed,TurretHood m_hood, TurretRotator m_turretRotator, Launcher m_launcher) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      new RevLauncher(launchSpeed, m_launcher),
      new SetHoodAngle(hoodAngle, m_hood),
      new SetTurretAngle(turretAngle, m_turretRotator));
  }
}
