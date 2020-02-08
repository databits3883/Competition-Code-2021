/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UpperStagingBelt extends SubsystemBase {
  private final CANSparkMax beltMotor = new CANSparkMax(Constants.topStagingBeltChannel, MotorType.kBrushless);
  
  private final DigitalInput ballSensor = new DigitalInput(Constants.upperStagingSensor);
  /**
   * Creates a new UpperStagingBelt.
   */
  public UpperStagingBelt() {}
  
  public void runBelt(){
    beltMotor.set(Constants.stagingSpeed);
  }
  public void stopBelt(){
    beltMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
