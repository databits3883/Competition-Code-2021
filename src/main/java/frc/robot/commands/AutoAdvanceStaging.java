/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomStagingBelt;

public class AutoAdvanceStaging extends CommandBase {
  BottomStagingBelt m_bottomStagingBelt;
  /**
   * Creates a new autoAdsvanceStaging.
   */
  public AutoAdvanceStaging(BottomStagingBelt stagingBelt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_bottomStagingBelt = stagingBelt;
    addRequirements(stagingBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_bottomStagingBelt.getSensorBottom() && !m_bottomStagingBelt.getSensorTop()){
      m_bottomStagingBelt.runBelt();
    }else{
      m_bottomStagingBelt.stopBelt();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_bottomStagingBelt.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
