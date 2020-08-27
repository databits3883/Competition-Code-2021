/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**Climb mechanism subsystem.
 * Handles the climb motor and brake rathcet servo.
 */

public class Climb extends SubsystemBase {
  Servo m_servo = new Servo(Constants.climbServoChannel);
  VictorSP m_winch =new VictorSP(Constants.liftWinchChannel);
  boolean isEngaged;
  double position;
  
  /**
   * Creates a new Climb subsystem instance. Engages the brake.
   */
  public Climb(){
    this.setEngaged(true);
  }

  /**
   * Set the state of the climb brake
   * @param engaged - true for locking the ratchet, false for unlocking.
   */
  public void setEngaged(boolean engaged){
    if(engaged){
      position= .7;
      isEngaged= true;
    }
    else {
      position =1;
      isEngaged= false;
    }
    m_servo.set(position);
  }
  /**
   * Is the locking servo currently engaged
   * @return false if ratchet open. True if locked.
   */
  public boolean isServoEngaged(){
    return isEngaged;
  }
  /**
   * Sets the winch motor to full speed lowering the hook.
   * {@link #stopHook() stopHook()} should be called after this.
   */
  public void lowerHook(){
    m_winch.set(1);
  }
  /**Sets the winch motor to raise the hook.
   * {@link #stopHook() stopHook()} should be called after this.
   */
  public void raiseHook(){
    m_winch.set(-.3);
  }
  /**Stops the winch motor from running.*/
  public void stopHook(){
    m_winch.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
