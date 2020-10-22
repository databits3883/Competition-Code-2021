/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;

public abstract class SparkMaxPIDSubsystem extends SubsystemBase {
  /**
   * Creates a new SparkMaxPIDSubsystem.
   */
  public SparkMaxPIDSubsystem(String name, CANSparkMax mainMotor, ControlType controlType, PIDTuningParameters tuning, double conversionFactor) {
    //set up conversion factor and accessor for the process variable
    DoubleSupplier processVariable;
    CANEncoder encoder = mainMotor.getEncoder();
    switch (controlType){
      case kPosition: encoder.setPositionConversionFactor(conversionFactor);
        processVariable = encoder::getPosition;
        break;
      case kVelocity: encoder.setVelocityConversionFactor(conversionFactor);
        processVariable = encoder::getVelocity;
        break;
      default: throw new IllegalArgumentException("SparkMaxPIDSubsystem doesn't yet support control type "+controlType.toString());
    }

    SparkMaxPIDController mainController =new SparkMaxPIDController(mainMotor,controlType,tuning);

    ShuffleboardTab tab = Shuffleboard.getTab(name);
    ShuffleboardLayout tuningLayout = tab.getLayout("tuning");
    mainController.addTuningToShuffleboard(tuningLayout);
    //add a graph with the setpoint and the current value
    tab.addDoubleArray("Process Variable vs Setpoint", ()->(new double[] {processVariable.getAsDouble(),mainController.getSetpoint()}))
      .withWidget(BuiltInWidgets.kGraph);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
