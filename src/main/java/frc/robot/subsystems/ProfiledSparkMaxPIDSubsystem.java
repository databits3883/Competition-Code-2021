/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SetpointVelocityLimiter;
/** Parent class for a subsystem based around a single profiled PID loop on a Spark Max motor */
public class ProfiledSparkMaxPIDSubsystem extends SparkMaxPIDSubsystem {
  SetpointVelocityLimiter m_limiter;
  /**
   * Creates a new ProfiledSparkMaxPIDSubsystem. Call in the constructor of a child class
   * @param name the name of the subsystem for use in Shuffleboard
   * @param mainMotor the Spark Max to control with a PID loop
   * @param controlType the control type the subsystem uses, must be {@link com.revrobotics.ControlType kPosition or kVelocity}
   * @param tuning the initial tuning for the subsystem's PID Controller
   * @param conversionFactor the conversion factor from rotations or rotations per minute dependiong on control type
   * @param min the minimum setpoint in converted units
   * @param max the maximum setpoint in converted units
   * @param maxVelocity the maximum rate of change for the setpoint
   * @param switchPolarity the polarity of attached limit switches
   */
  public ProfiledSparkMaxPIDSubsystem(String name, CANSparkMax mainMotor, ControlType controlType, PIDTuningParameters tuning, double conversionFactor, double min, double max, double maxVelocity, LimitSwitchPolarity switchPolarity) {
    //runs SparkMaxPIDController constructor
    super(name,mainMotor,controlType,tuning,conversionFactor,min,max,switchPolarity);
    //creates the velocity limiter for use in periodic()
    m_limiter = new SetpointVelocityLimiter(maxVelocity);
  }

  @Override
  void setSetpointInternal(double newSetpoint) {
    newSetpoint = MathUtil.clamp(newSetpoint, m_setpointMin, m_setpointMax);
    //sets the target setpoint to the limiter, the setpoint is sent to the motor controller in periodic()
    m_limiter.setTarget(newSetpoint);
  }

  @Override
  public void periodic() {
    // periodically update the motor controller setpoint to the limited value
    m_mainController.setSetpoint(m_limiter.get());
  }

  public void setSetpointStep(double newSetpoint){
    newSetpoint = MathUtil.clamp(newSetpoint, m_setpointMin, m_setpointMax);
    m_limiter.setWithoutRamp(newSetpoint);
    m_mainController.setSetpoint(newSetpoint);
    m_mainController.reset();
    m_setpointEntry.setDouble(newSetpoint);
  }

  @Override
  public void holdCurrentPosition() {
    double currentPosition = m_processVariable.getAsDouble();
    setSetpointStep(currentPosition);
  }
  @Override
  public double getCurrentError(){
    return m_limiter.getCurrentTarget() - m_processVariable.getAsDouble();
  }

  @Override 
  public boolean onTarget() {
    return Math.abs(m_limiter.getCurrentTarget() - m_processVariable.getAsDouble())<=m_tolerance;
  }
}
