/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final VictorSP intakeMotor = new VictorSP(Constants.intakeRetrieveChannel);
  private final CANSparkMax extenderMotor = new CANSparkMax(Constants.intakeExtenderChannel, MotorType.kBrushed);

  private final CANDigitalInput extendedLimitSwitch = new CANDigitalInput(extenderMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyClosed);
  private final CANDigitalInput retractedLimitSwitch = new CANDigitalInput(extenderMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyClosed);
  /**
   * Creates a new Intake.
   */
  public Intake() {
    
    Shuffleboard.getTab("Variables").addBoolean("extended switch",extendedLimitSwitch::get);
    Shuffleboard.getTab("Variables").addBoolean("retracted switch", retractedLimitSwitch::get);
  }
  public void intake(){
    intakeMotor.set(Constants.intakeSpeed);
  }
  public void stop(){
    intakeMotor.set(0);
  } 
  public void Outake(){
    intakeMotor.set(-Constants.intakeSpeed);
  }

  public void retractIntake(){
    extenderMotor.set(Constants.Raiseintakespeed);
  }
  public void extendIntake(){
    extenderMotor.set(Constants.loweringIntakeSpeed);
  }
  public void stopExtension(){
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
