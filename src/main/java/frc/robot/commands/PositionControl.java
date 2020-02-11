/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSpinner.WheelColor;

public class PositionControl extends CommandBase {
    ControlPanelSpinner.WheelColor targetUnderSensor;
    private final ControlPanelSpinner m_controlPanelSpinner;
    boolean spinLeft; 

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
        break;
      case 'Y':
        targetUnderSensor = WheelColor.kGreen;
        break;
      case 'B':
        targetUnderSensor = WheelColor.kRed;
        break;
      case 'G':
        targetUnderSensor = WheelColor.kYellow;
        break;
      default:
        break;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_controlPanelSpinner.getColor()==targetUnderSensor);
  }
}
