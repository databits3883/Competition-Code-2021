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
/**Limelight tilter servo subsystem */
public class LimelightServo extends SubsystemBase {
  Servo m_servo = new Servo(Constants.cameraServo);
  SetpointVelocityLimiter velocityLimiter = new SetpointVelocityLimiter(45);
  /**
   * Creates a new LimelightServo instance. sets initial servo position.
   */
  public LimelightServo() {
    m_servo.setPosition(1);
  }
  /**Set the target position for the servo
   * @param position angle from 0 to 180.
   */
  public void setPosition(double position){
    position = MathUtil.clamp(position, 0, 180);
    velocityLimiter.setTarget(position);
  }
  /**Adds an angle to the current position for the new setpoint
   * @param delta angle to add to current target
   */
  public void deltaPosition(double delta){
    setPosition(getAngle()+delta);
    
  }
  /** gets current angle of the camera
   * @return target angle of the servo.
   */
  public double getAngle(){
    return velocityLimiter.getCurrentSetpoint();
  }
  //TODO: Check this doc for accuracy
  /**gets angle to the horizon 
   * @return angle to the horizon from the camera, where positive is up.
  */
  public double getAngleToTarget(){
    return getAngle()*-1.0+90.0;
  }
  /**updates servo position to velocity limited target  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_servo.setAngle(velocityLimiter.get());
    
    
  }
}
