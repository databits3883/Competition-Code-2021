/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.TurretHood;
import frc.robot.subsystems.TurretLauncher;
import frc.robot.subsystems.TurretCameraAim;
import frc.robot.subsystems.TurretRotator;

public class AcquireTarget extends CommandBase {
  TurretCameraAim m_LimelightServo;
  TurretRotator m_tTurretRotator;
  TurretLauncher m_launcher;
  TurretHood m_hood;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry filterLengthEntry;
  NetworkTableEntry limelightTolerance, angleTolerance;
  NetworkTableEntry Pipeline;
  double filterLength;
  LinearFilter limeLightFilter = LinearFilter.movingAverage(10);
  /**
   * Creates a new AcquireTarget.
   */
  public AcquireTarget(TurretCameraAim limelightServo, TurretRotator turretRotator, TurretHood hood, TurretLauncher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LimelightServo = limelightServo;
    m_tTurretRotator = turretRotator;
    m_hood = hood;
    m_launcher = launcher;
    addRequirements(m_LimelightServo,m_tTurretRotator);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    Pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("getPipeline");
    filterLengthEntry = Shuffleboard.getTab("tuningLime").add("filter length",10).getEntry();

    limelightTolerance = Shuffleboard.getTab("tuningLime").add("limelight tolerance",1).getEntry();
    angleTolerance = Shuffleboard.getTab("tuningLime").add("turret angle tolerance",.5).getEntry();

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    verticalAim();
    horizontalAim();
    double angle = m_LimelightServo.getAngleToTarget();
    hoodAim(angle);
    launcherPrime(angle);

    //System.out.println("angle: "+m_LimelightServo.getAngleToTarget());
    if(filterLengthEntry.getDouble(filterLength)!=filterLength){
      filterLength = filterLengthEntry.getDouble(filterLength);
      limeLightFilter = LinearFilter.movingAverage((int)filterLength);
    }
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
    double xOffset = limeLightFilter.calculate(tx.getDouble(0));
    if(horizontalOnTarget){
      if (Math.abs(xOffset)<limelightTolerance.getDouble(1)) xOffset = 0;
      m_tTurretRotator.changeAngle(-xOffset);
      horizontalOnTarget = false;
    }
    horizontalOnTarget = Math.abs(m_tTurretRotator.getCurrentError())<angleTolerance.getDouble(1); 
  }
  void hoodAim(double angle){
    m_hood.setAngle(
      //angle>=4.577?25.777:36.5-1.69*angle-0.143*angle*angle
      38.8 - 1.17*angle + 0.0785*angle*angle -(1.94E-3)*angle*angle*angle
    );
  }
  void launcherPrime(double angle){
    m_launcher.setSpeed(MathUtil.clamp((-84.4+3.61*angle-0.217*angle*angle+(4.37E-3)*angle*angle*angle),-100,0));
  }

  boolean horizontalOnTarget = true;
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
      verticalOnTarget = true;
      System.out.println("ended");
      horizontalOnTarget = true;
     if (interrupted){
      m_launcher.setSpeed(0);
     }
     NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(3);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
