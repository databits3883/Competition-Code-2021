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

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeChannel, MotorType.kBrushed);
  private final CANSparkMax extenderMotor = new CANSparkMax(Constants.intakeExtenderChannel, MotorType.kBrushless);

  private final DigitalInput extendedLimitSwitch = new DigitalInput(Constants.extendedLimitSwitchChannel);
  private final DigitalInput retractedLimitSwitch = new DigitalInput(Constants.retractedLimitSwitchChannel);
  /**
   * Creates a new Intake.
   */
  public Intake() {

  }
  public void intake(){
    intakeMotor.set(1);
  }
  public void stop(){
    intakeMotor.set(0);
  } 

  public void moveWinchUp(){
    extenderMotor.set(.7);
  }
  public void moveWinchDown(){
    extenderMotor.set(-.5);
  }
  public void stopWinch(){
    extenderMotor.set(0);
  }
  public boolean getExtendLimit(){
    return extendedLimitSwitch.get();
  }
  public boolean getretractedLimit(){
    return retractedLimitSwitch.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
