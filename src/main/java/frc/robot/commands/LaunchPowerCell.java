/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Staging;


public class LaunchPowerCell extends CommandBase {
  Staging staging;
  /**
   * Creates a new LaunchPowerCell.
   */
  public LaunchPowerCell(Staging m_staging) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_staging = staging;
    
    addRequirements(staging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    staging.RunStaging();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    staging.StopStaging();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !staging.GetTopSensor();
  }
}
