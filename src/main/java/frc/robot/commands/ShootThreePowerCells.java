/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Variables;
import frc.robot.subsystems.BottomStagingBelt;
import frc.robot.subsystems.UpperStagingBelt;



public class ShootThreePowerCells extends CommandBase {
  UpperStagingBelt m_upperStagingBelt;
  BottomStagingBelt m_bottomStagingBelt;
  int numberOfPowerCells;
  /**
   * Creates a new shootThreeBalls.
   */
  public ShootThreePowerCells(UpperStagingBelt upperStagingBelt, BottomStagingBelt bottomStagingBelt) {
    addRequirements(upperStagingBelt, bottomStagingBelt);
    m_upperStagingBelt = upperStagingBelt;
    m_bottomStagingBelt = bottomStagingBelt;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    numberOfPowerCells = Variables.getInstance().getContainedPowerCells();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_upperStagingBelt.runBelt(); 
    m_bottomStagingBelt.runBelt();
    System.out.println("running shoot 3");
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_bottomStagingBelt.stopBelt();
    m_upperStagingBelt.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numberOfPowerCells -3 >=  Variables.getInstance().getContainedPowerCells();
  }
}
