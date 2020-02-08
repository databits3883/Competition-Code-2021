/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControlPanelSpinner extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(Constants.controlWheelChannel, MotorType.kBrushless);
  ColorSensorV3 sensor = new ColorSensorV3(Constants.colorSensorPort);
  /**
   * Creates a new ControlPanelSpinner.
   */
  public ControlPanelSpinner() {

  }

  public WheelColor getColor(){
    Color measured = sensor.getColor();
    if(measured.red>Constants.redThreshold){
      if(measured.green>Constants.greenThreshold){
        return WheelColor.kYellow;
      }else{
        return WheelColor.kRed;
      }
    }else{
      if(measured.blue > Constants.blueThreshold){
        return WheelColor.kBlue;
      }else{
        return WheelColor.kGreen;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum WheelColor{
    kRed, kYellow, kBlue, kGreen
  }
}
