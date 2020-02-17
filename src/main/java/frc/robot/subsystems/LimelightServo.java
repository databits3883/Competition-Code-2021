/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SetpointVelocityLimiter;

public class LimelightServo extends SubsystemBase {
  Servo m_servo = new Servo(Constants.camerServo);
  SetpointVelocityLimiter velocityLimiter = new SetpointVelocityLimiter(90);
  /**
   * Creates a new LimelightServo.
   */
  public LimelightServo() {
    m_servo.setPosition(1);
  }
  public void setPosition(double position){
    velocityLimiter.setTarget(position);
  }
  public void deltaPosition(double delta){
    setPosition(getAngle()+delta);
    
  }
  public double getAngle(){
    return velocityLimiter.getCurrentSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_servo.setAngle(velocityLimiter.get());
  }
}
