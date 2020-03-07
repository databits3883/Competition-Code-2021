/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LimelightServo;

public class AutoAiming extends CommandBase {
  LimelightServo m_limelightservo;
  Hood m_hood;
  
  NetworkTableEntry tv;
  Launcher m_launcher;
  

  /**
   * Creates a new AutoAiming.
   */
  public AutoAiming(LimelightServo limelightservo,Launcher launcher, Hood hood) {
    m_limelightservo = limelightservo;
    addRequirements(m_limelightservo);
    m_launcher = launcher;
    addRequirements(m_launcher);
    m_hood = hood;
    addRequirements(m_hood);
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  double GetAngle(){
    return m_limelightservo.getAngleToTarget();
  }
  // boolean IsValid(){
  //   return tv.isValid();
  // }
  double la;
  double lv; 
  double dx;
  double dy = 5.1;
  
  
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    la = Math.toRadians(GetAngle());
    dx = 5.1/(Math.tan(la));

    
    m_hood.setAngle((57.3-la)+10);
    // if (IsValid())
   // m_launcher.setSpeed((dx*3.1321)/(Math.cos(GetAngle())*Math.sqrt(dx*Math.tan(GetAngle())-dy))/-2.697);
   lv = (dx*5.67)/((Math.cos(la+10))*Math.sqrt(dx*(Math.tan(la+10))-dy))*2.697;
   //System.out.println((Math.cos(la))*Math.sqrt(dx*(Math.tan(la))-5.1));
   
   //System.out.println(dx*(Math.tan(la))-5.1);
   //System.out.println("x: "+dx+"theta radians: "+la+"theta degrees: "+GetAngle());
   //double d1 = (32.2*(Math.cos(la)*Math.cos(la)))/(2*(dx*dx));
   //double d2 = (Math.sin(la)*Math.cos(la))/dx;
   //double NewVelocity = Math.sqrt(5.1/(d1+d2));
   //System.out.println(NewVelocity);

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
