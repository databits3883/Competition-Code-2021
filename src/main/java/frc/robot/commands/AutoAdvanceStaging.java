/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Staging;

public class AutoAdvanceStaging extends CommandBase {
  Staging m_staging;
  /**
   * Creates a new autoAdsvanceStaging.
   */
  public AutoAdvanceStaging(Staging stagingBelt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_staging = stagingBelt;
    addRequirements(stagingBelt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  boolean cellAtBottom = false;
  boolean wasCellAtBottom = false;

  boolean cellAtMiddle = false;
  boolean wasCellAtMiddle= false;

  boolean isBeltRunning;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wasCellAtBottom =cellAtBottom;
    cellAtBottom = m_staging.GetBottomSensor();

    wasCellAtMiddle = cellAtMiddle;
    cellAtMiddle = m_staging.GetMiddleSensor();

    if(cellAtBottom && !wasCellAtBottom && !isBeltRunning){
      isBeltRunning = true;
    }
    if(isBeltRunning && !wasCellAtMiddle && cellAtMiddle){
      isBeltRunning = false;
    }
    if(m_staging.GetTopSensor()) isBeltRunning = false;

    if(isBeltRunning){
      m_staging.RunStaging();
      m_staging.Jostle();
    }else{
      m_staging.StopStaging();
      m_staging.StopJostle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_staging.StopJostle();
    m_staging.StopStaging();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
