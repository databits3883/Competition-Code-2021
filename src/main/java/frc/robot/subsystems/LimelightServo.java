/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.SetpointVelocityLimiter;

public class LimelightServo extends SubsystemBase {
  Servo m_servo = new Servo(Constants.camerServo);
  SetpointVelocityLimiter velocityLimiter = new SetpointVelocityLimiter(45);
  /**
   * Creates a new LimelightServo.
   */
  public LimelightServo() {
    m_servo.setPosition(1);
  }
  public void setPosition(double position){
    position = MathUtil.clamp(position, 0, 180);
    velocityLimiter.setTarget(position);
  }
  public void deltaPosition(double delta){
    setPosition(getAngle()+delta);
    
  }
  public double getAngle(){
    return velocityLimiter.getCurrentSetpoint();
  }
  public double getAngleToTarget(){
    return getAngle()*-1.0+90.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_servo.setAngle(velocityLimiter.get());
  }
}
