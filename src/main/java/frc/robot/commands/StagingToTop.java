/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Staging;


public class StagingToTop extends CommandBase {
  Staging staging;
  /**
   * Creates a new StagingToTop.
   */
  public StagingToTop( Staging m_staging ) {
     staging = m_staging;
    addRequirements(m_staging);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    staging.Jostle();
    staging.MovePowerCelltoLauncher();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    staging.StopJostle();
    staging.MovePowerCellStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return staging.GetTopSensor();
  }
}
