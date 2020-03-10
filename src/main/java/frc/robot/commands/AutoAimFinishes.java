/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LimelightServo;
import frc.robot.subsystems.TurretRotator;


public class AutoAimFinishes extends AcquireTarget {
  LimelightServo m_limelightServo;
  TurretRotator m_turretRotator;
  Hood m_hood;
  Launcher m_launcher;
  /**
   * Creates a new AutoAimFinishes.
   */
  public AutoAimFinishes(LimelightServo limelightServo, TurretRotator turretRotator, Hood hood, Launcher launcher) {
    super(limelightServo, turretRotator, hood, launcher);
    addRequirements(limelightServo);
    m_LimelightServo=limelightServo;
    addRequirements(turretRotator);
    m_tTurretRotator = turretRotator;
    addRequirements(hood);
    m_hood = hood;
    addRequirements(launcher);
    m_launcher = launcher;

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_launcher.atSpeed()&&m_hood.atAngle()&&m_turretRotator.atTarget();
  }
}
