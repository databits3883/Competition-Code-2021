/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSpinner.WheelColor;

public class PositionControl extends CommandBase {
    private final ControlPanelSpinner m_controlPanelSpinner;
    ControlPanelSpinner.WheelColor targetUnderSensor;
    int targetindex;
    int currentindex;
    boolean[][] directions = {{true, false, false, true},{true, true, false, false},{false,true,true,false},{false,false,true,true}};

  /**
   * Creates a new PositionControl.
   */
  public PositionControl(ControlPanelSpinner controlPanelSpinner) {
    m_controlPanelSpinner = controlPanelSpinner;
    addRequirements(controlPanelSpinner);
  }

    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(DriverStation.getInstance().getGameSpecificMessage().charAt(0)){
      case 'R':
        targetUnderSensor = WheelColor.kBlue;
        targetindex =2;
        break;
      case 'Y':
        targetUnderSensor = WheelColor.kGreen;
        targetindex =3;
        break;
      case 'B':
        targetUnderSensor = WheelColor.kRed;
        targetindex =0;
        break;
      case 'G':
        targetUnderSensor = WheelColor.kYellow;
        targetindex =1;
        break;
      default:
        
        break;
    }
    switch (m_controlPanelSpinner.getColor()){
      case kBlue:
      currentindex=0;
      break;
      case kGreen:
      currentindex=1;
      break;
      case kRed:
      currentindex=2;
      break;
      case kYellow:
      currentindex=3;
    break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( m_controlPanelSpinner.getEngaged()){
      if (directions [targetindex][currentindex]){
      m_controlPanelSpinner.spinClockwise();
      }
      else{
      m_controlPanelSpinner.spinCounterclockwise();
      }
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_controlPanelSpinner.getColor()==targetUnderSensor){
      m_controlPanelSpinner.stopSpin();
      return true;
    }
    else{
      return false;
    }
  }
}
