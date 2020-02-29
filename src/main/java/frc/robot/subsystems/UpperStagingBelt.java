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
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UpperStagingBelt extends SubsystemBase {
  private final VictorSP beltMotor = new VictorSP(Constants.upperIntakeChannel);
  
  private final DigitalInput ballSensor = new DigitalInput(Constants.upperStagingSensor);
  /**
   * Creates a new UpperStagingBelt.
   */
  public UpperStagingBelt() {}
  
  public void runBelt(){
    beltMotor.set(-1);
    System.out.println("running upper belt");
  }
  public void stopBelt(){
    beltMotor.stopMotor();
  }
  public void outTake(){
    beltMotor.set(0.4);
  }

  public boolean isBallPresent(){
    return !ballSensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
