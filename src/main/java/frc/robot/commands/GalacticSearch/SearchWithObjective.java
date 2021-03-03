// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GalacticSearch;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Variables;
import frc.robot.commands.BallFollowing;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Odometry;

public class SearchWithObjective extends CommandBase {
  /** Creates a new SearchWithObjective. */
  Drivetrain m_drivetrain;
  Odometry m_odometry;
  Character m_endpointAxis;
  Double xOffset;
  Double yOffset;
  double m_targetX;
  double m_targetY;

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry Pipeline;
  
  public SearchWithObjective(/*endpoint*/Drivetrain drivetrain, Character endpointAxis, double targetX, double targetY ) {
    // Use addRequirements() here to declare subsystem dependencies.

    //endpointAxis: y for y, x for x, b for both
    m_drivetrain = drivetrain;
    
    m_endpointAxis = endpointAxis;
    m_targetX = targetX;
    m_targetY = targetY;

    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    Pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tv.getBoolean(false)){
      pointToAngle(getAngleToTarget(m_targetX, m_targetY), .2, 240);

    }
    else{
      chaseBall(.2);

    }
  }
  
  public double getAngleToTarget(Double targetX, double targetY){
    double robotX = m_drivetrain.getRobotPose().getTranslation().getX();
    double robotY = m_drivetrain.getRobotPose().getTranslation().getY();

    double offX = targetX - robotX;
    double offY = targetY - robotY;

    double angle = Math.atan(offX/offY)*(180/Math.PI);
    if(Variables.getInstance().getGyroAngle()>180){
      return angle + 180;
    }
    else{
      return angle + 180;
    }
  }
  public void pointToAngle(double targetAngle, double speed, double p){
    double robotAngle =  Variables.getInstance().getGyroAngle();
    double error = (targetAngle - robotAngle)/p;
    m_drivetrain.ArcadeDrive(error, speed);


  }

  public void chaseBall(double speed){
    double error = tx.getDouble(0)/30;
    m_drivetrain.ArcadeDrive(error,  speed);
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
  public void end(boolean interrupted) {
    if(tv.getBoolean(false)){
      CommandScheduler.getInstance().schedule(new BallFollowing().andThen(new SearchWithObjective(m_drivetrain, m_endpointAxis,m_targetX, m_targetY)));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if past point{
      //return true}
    //if not{
      switch(m_endpointAxis){
        case 'x': if(m_drivetrain.getRobotPose().getTranslation().getX()> m_targetX){
          return true;
        }
        case 'y': if(m_drivetrain.getRobotPose().getTranslation().getY()>m_targetY){
          return false;
        }
        case 'b': if(m_drivetrain.getRobotPose().getTranslation().getX()>m_targetX && m_drivetrain.getRobotPose().getTranslation().getY() > m_targetY){
          return true;
        }
        default: return false;
      }
    
  }
}
