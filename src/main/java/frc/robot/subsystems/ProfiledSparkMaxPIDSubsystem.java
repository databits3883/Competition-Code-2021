/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SetpointVelocityLimiter;

public class ProfiledSparkMaxPIDSubsystem extends SparkMaxPIDSubsystem {
  SetpointVelocityLimiter m_limiter;
  /**
   * Creates a new ProfiledSparkMaxPIDSubsystem.
   */
  public ProfiledSparkMaxPIDSubsystem(String name, CANSparkMax mainMotor, ControlType controlType, PIDTuningParameters tuning, double conversionFactor, double min, double max, double maxVelocity) {
    super(name,mainMotor,controlType,tuning,conversionFactor,min,max);
    m_limiter = new SetpointVelocityLimiter(maxVelocity);
  }

  @Override
  void setSetpointInternal(double newSetpoint) {
    newSetpoint = MathUtil.clamp(newSetpoint, m_setpointMin, m_setpointMax);
    m_limiter.setTarget(newSetpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
