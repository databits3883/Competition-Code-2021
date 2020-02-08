/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BottomStagingBelt extends SubsystemBase {
  private final CANSparkMax beltMotor = new CANSparkMax(Constants.bottomStagingBeltChannel, MotorType.kBrushless);

  private final DigitalInput lowerSwitch = new DigitalInput(Constants.lowerStagingStartSensor);
  private final DigitalInput upperSwitch = new DigitalInput(Constants.lowerStagingEndSensor);
  /**
   * Creates a new BottomStagingBelt.
   */
  public BottomStagingBelt() {
    beltMotor.setIdleMode(IdleMode.kBrake);

  }
  public boolean getSensorBottom(){
    return lowerSwitch.get();
  }
  public boolean getSensorTop(){
    return upperSwitch.get();
  }
  public void runBelt(){
    beltMotor.set(.25);
  }
  public void stopBelt(){
    beltMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
