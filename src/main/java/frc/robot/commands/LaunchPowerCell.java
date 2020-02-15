/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomStagingBelt;
import frc.robot.subsystems.UpperStagingBelt;

public class LaunchPowerCell extends CommandBase {
  UpperStagingBelt m_upperStagingBelt;
  BottomStagingBelt m_bottomStagingBelt;
  /**
   * Creates a new LaunchPowerCell.
   */
  public LaunchPowerCell(UpperStagingBelt upperStagingBelt, BottomStagingBelt bottomStagingBelt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_upperStagingBelt = upperStagingBelt;
    m_bottomStagingBelt = bottomStagingBelt;
    addRequirements(upperStagingBelt, bottomStagingBelt );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_upperStagingBelt.runBelt();
    m_bottomStagingBelt.runBelt();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_upperStagingBelt.stopBelt();
    m_bottomStagingBelt.runBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_upperStagingBelt.isBallPresent();
  }
}
