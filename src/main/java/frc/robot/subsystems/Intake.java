/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeChannel, MotorType.kBrushless);
  private final CANSparkMax extenderMotor = new CANSparkMax(Constants.intakeExtenderChannel, MotorType.kBrushless);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
