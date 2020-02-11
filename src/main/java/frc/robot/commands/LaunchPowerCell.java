/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperStagingBelt;

public class LaunchPowerCell extends CommandBase {
  UpperStagingBelt m_UpperStagingBelt;
  /**
   * Creates a new LaunchPowerCell.
   */
  public LaunchPowerCell(UpperStagingBelt upperStagingBelt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_UpperStagingBelt = upperStagingBelt;
    addRequirements(upperStagingBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_UpperStagingBelt.runBelt();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_UpperStagingBelt.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_UpperStagingBelt.isBallPresent();
  }
}
