/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.PIDTuningParameters;

public class TurretRotator extends ProfiledSparkMaxPIDSubsystem {
  static double gearRatio = (1.0/15.0);
  static double degreeRatio = (360.0);
  static double conversionFactor = gearRatio*degreeRatio;

  /**
   * Creates a new TurretRotator.
   */
  public TurretRotator() {
    super("Turret Rotator",
     new CANSparkMax(Constants.turretRotationChannel, MotorType.kBrushless), 
     ControlType.kPosition,
      new PIDTuningParameters(0.04, 0, 0),
       conversionFactor,
      0, Constants.maxTurretAngle,
      Constants.maxTurretVelocity,LimitSwitchPolarity.kNormallyOpen);
    setTolerance(0.75);
    setCurrentPosition();
  }
  
  public void setAngle(double newSetpoint){
    setSetpoint(newSetpoint);
  }
  public void setAngleStep(double newSetpoint){
    setSetpointStep(newSetpoint);
  }
  public void changeAngle(double angleDelta){
    //System.out.println(angleDelta);
    setAngle(m_limiter.getCurrentTarget()+ angleDelta);
  }

  public void setCurrentPosition(){
    holdCurrentPosition();
  }
  public double getCurrentAngle(){
    return m_processVariable.getAsDouble();
  }
 
  public boolean atTarget(){
    return onTarget();
  }

    
  
  @Override
  public void periodic() {
     // This method will be called once per scheduler run
    super.periodic();
    if(DriverStation.getInstance().isDisabled()) checkLimitSwitches();
  }
}
