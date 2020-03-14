/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Variables;

import frc.robot.subsystems.Staging;


public class ManualLaunch extends CommandBase {
  Staging m_staging;
  boolean ballEntered;
  /**
   * Creates a new ManualLaunch.
   */
  public ManualLaunch(Staging staging) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_staging = staging;
    addRequirements(staging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballEntered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Variables.getInstance().getShooterEnabled()&&Variables.getInstance().getShooterAtSpeed()){
      m_staging.RunStaging();
    }else{
      m_staging.StopStaging();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_staging.StopStaging();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return ballEntered && !m_upperStagingBelt.isBallPresent();
    return false;
  }
}
