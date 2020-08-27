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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**Control panel Spinner Class. Manages spinner wheel and color sensor.
 * <em>May be</em> depracated.
*/
public class ControlPanelSpinner extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(Constants.controlWheelChannel, MotorType.kBrushless);
  private final DigitalInput pressedBackSwitch = new DigitalInput(Constants.controlPanelPressedChannel);
  ColorSensorV3 sensor = new ColorSensorV3(Constants.colorSensorPort);
  /**
   * Creates a new ControlPanelSpinner instance.
   */
  public ControlPanelSpinner() {
    
  }
  /**Spins the color wheel counterclockwise when engaged with the spinner wheel. {@link #stopSpin() stopSpin()} should be called later.*/
  public void spinCounterclockwise(){
    motor.set(1);
  }
  /**Spins the color wheel clockwise when engaged with the spinner wheel. {@link #stopSpin() stopSpin()} should be called later.*/
  public void spinClockwise(){
    motor.set(-1);
  }
  /**Stops the color wheel spinning motor. */
  public void stopSpin(){
    motor.stopMotor();
  }

  /**
   * Get whether the spinner motor is engaged with the color wheel in order to spin it.
   * @return true if ready to spin the color wheel.
   */
  public boolean getEngaged(){
    return pressedBackSwitch.get();
  }

  /**
   * get the color under the color sensor based on {@link frc.robot.Constants#redThreshold Constants} 
   * @return kYellow, kRed, kBlue, or kGreen
   */
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
    //System.out.println(getColor());
    // This method will be called once per scheduler run
  }
  /**Enum representing colors on the color wheel.
   * <dl>
   * <dt><b>Values:</b></dt><dd>kRed, kYellow, kBlue, kGreen</dd>
   * </dl>
   */
  public enum WheelColor{
    kRed, kYellow, kBlue, kGreen
  }
}
