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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.util.PIDTuningParameters;

public class TurretHood extends ProfiledSparkMaxPIDSubsystem {
  /**
  * Creates a new Hood.
  */  

  private static final double conversionFactor = 16.0*360.0/264.0/50.0;

  public TurretHood() {
    super("Turret Hood", 
    new CANSparkMax(Constants.turretHoodChannel, MotorType.kBrushless), 
    ControlType.kPosition, 
    new PIDTuningParameters(0.02,0,0),
    conversionFactor, 
    Constants.minimumHoodAngle, Constants.maximumHoodAngle, Constants.maxHoodVelocity,LimitSwitchPolarity.kNormallyClosed);

    setTolerance(1.5);
  }

  public boolean atAngle(){
    return onTarget();
  }

  @Override
  public void periodic() {
    checkLimitSwitches();
    super.periodic();
    // This method will be called once per scheduler run
  }
  public void setAngle(double newAngle){
    setSetpoint(newAngle);
  }
  
  public void changeAngle(double angleDelta){
    setAngle(m_mainController.getSetpoint()+ angleDelta);
  }
  

  public void setCurrentPosition(){
    holdCurrentPosition();
  }
  public double LaunchAngle(){
    return (57.3-(m_processVariable.getAsDouble()-17));
  }
}
