/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;

public abstract class SparkMaxPIDSubsystem extends SubsystemBase {
  /**
   * Creates a new SparkMaxPIDSubsystem.
   */
  public SparkMaxPIDSubsystem(String name, CANSparkMax mainMotor, SparkMaxPIDController mainController, ControlType controlType, PIDTuningParameters tuning) {
    mainController=new SparkMaxPIDController(mainMotor,controlType,tuning);

    ShuffleboardLayout tuningLayout = Shuffleboard.getTab(name).getLayout("tuning");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
