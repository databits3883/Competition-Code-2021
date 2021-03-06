/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretCameraAim;
import frc.robot.subsystems.TurretRotator;

public abstract class BallFollowing extends CommandBase {
  Drivetrain m_drivetrain;
  TurretRotator m_turretrotator;
  TurretCameraAim m_limelightservo;
  Intake m_intake;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  NetworkTableEntry Pipeline;
  PIDController turnpid = new PIDController(0.01,0,0);
  
  PIDController speedpid = new PIDController(0.01,0,0);
  
  /**
   * Creates a new BallFollowing.
   */
  public BallFollowing(Drivetrain drivetrain, TurretRotator turretrotator, TurretCameraAim limelightservo, Intake intake) {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta");
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    Pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    m_turretrotator = turretrotator;
    addRequirements(m_turretrotator);
    m_limelightservo = limelightservo;
    addRequirements(m_limelightservo);
    m_intake = intake;
    addRequirements(m_intake);
    // SendableRegistry.setName(turnpid, "turn pid");
    // SendableRegistry.setName(speedpid, "speed pid");
    // Shuffleboard.getTab("BallFollowing").add(turnpid);
    // Shuffleboard.getTab("BallFollowing").add(speedpid);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  boolean verticalOnTarget = true;

  void verticalAim(){
    
    if(verticalOnTarget){
      double yOffset = ty.getDouble(0)*(1.0/10.0);
      m_limelightservo.deltaPosition(-yOffset);
      verticalOnTarget = false;
      //System.out.println(yOffset);
    }
    verticalOnTarget = true;

  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pipeline.setNumber(3);
    
    
    m_turretrotator.setAngleStep(277);
    m_limelightservo.setPosition(139);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shouldTurn()&& !shouldDrive()){
      m_drivetrain.ArcadeDrive(-turnpid.calculate(tx.getDouble(0), 0), 0);
    }
    else if(shouldDrive()){
      m_drivetrain.ArcadeDrive(-turnpid.calculate(tx.getDouble(0), 0), calculatespeed());
    }
    else {
      noTargetDrive();
    }

    //verticalAim();
  }

   boolean shouldTurn(){
      return (tv.getDouble(0) == 1 && ta.getDouble(0)>.14);
   } 
   boolean shouldDrive(){
     return( shouldTurn() &&  Math.abs(tx.getDouble(0))<5);
   }
   abstract double calculatespeed();
   abstract void noTargetDrive();


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tv.getDouble(0)==0;
  }
}
