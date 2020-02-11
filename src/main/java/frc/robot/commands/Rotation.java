/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSpinner.WheelColor;

public class Rotation extends CommandBase {
  private final ControlPanelSpinner m_controlPanelSpinner;
  WheelColor color2;
  int change = 0;

  /**
   * Creates a new ColorWheelCommand.
   */
  public Rotation(ControlPanelSpinner controlPanelSpinner) {
    m_controlPanelSpinner = controlPanelSpinner;
    addRequirements(controlPanelSpinner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_controlPanelSpinner.getColor();
    color2 = m_controlPanelSpinner.getColor();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   WheelColor color1 = m_controlPanelSpinner.getColor();
   if( color1 == color2){
    m_controlPanelSpinner.spinCounterclockwise();
   }
   else{ 
    change = change + 1;
     color2 = m_controlPanelSpinner.getColor();
    m_controlPanelSpinner.spinCounterclockwise();

   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(change == 25){
      m_controlPanelSpinner.stopSpin();
    return true; 
    }
    else{
     return false; 
    }
  
  }
  
  
}
