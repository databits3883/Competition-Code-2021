// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GalacticSearch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Odometry;

public class SearchWithObjective extends CommandBase {
  /** Creates a new SearchWithObjective. */
  Drivetrain m_drivetrain;
  Odometry m_odometry;
  Character m_endpointAxis;
  public SearchWithObjective(/*endpoint*/Drivetrain drivetrain,Odometry odometry, Character endpointAxis, double targetX, double targetY ) {
    // Use addRequirements() here to declare subsystem dependencies.

    //endpointAxis: y for y, x for x, b for both
    m_drivetrain = drivetrain;
    m_odometry = odometry;
    m_endpointAxis = endpointAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_endpointAxis){
      case 'x':
      case 'y':
      case 'b':
    }
  }
  public boolean detectPastX(){


  };
  public double getAngleToTarget{
    
  }
  //get targetx - robotx
  //same with y
  //angle = arctan(xoffset/yoffset)
  //if pointed backwards add 180 degrees
  //point to angle
  //drive forward
  //if (tv == 1){
    //chase ball

  //}
  //if (tv == 0){
//point to angle
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if past point{
      //return true}
    //if not{
    return false;
  }
}
