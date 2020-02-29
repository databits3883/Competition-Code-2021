/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightServo;
import frc.robot.subsystems.TurretRotator;

public class AcquireTarget extends CommandBase {
  LimelightServo m_LimelightServo;
  TurretRotator m_tTurretRotator;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  /**
   * Creates a new AcquireTarget.
   */
  public AcquireTarget(LimelightServo limelightServo, TurretRotator turretRotator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LimelightServo = limelightServo;
    m_tTurretRotator = turretRotator;
    addRequirements(m_LimelightServo,m_tTurretRotator);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    verticalAim();
    horizontalAim();
    System.out.println("distance: "+(8.0+(3.75/12.0)-3.0)/Math.tan(Math.toRadians(m_LimelightServo.getAngleToTarget())));
  }
  boolean verticalOnTarget = true;
  void verticalAim(){
    
    if(verticalOnTarget){
      double yOffset = ty.getDouble(0)*(1.0/10.0);
      m_LimelightServo.deltaPosition(-yOffset);
      verticalOnTarget = false;
      //System.out.println(yOffset);
    }
    verticalOnTarget = true;

  }
  void horizontalAim(){
    if(horizontalOnTarget){
      double xOffset = tx.getDouble(0)*(1.0);
      m_tTurretRotator.changeAngle(-xOffset);
      horizontalOnTarget = false;
    }
    horizontalOnTarget = Math.abs(m_tTurretRotator.getCurrentError())<1; 
  }

  boolean horizontalOnTarget = true;
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
      verticalOnTarget = true;
      System.out.println("ended");
      horizontalOnTarget = true;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
