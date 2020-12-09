/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;
import frc.robot.util.NetworkTablesUpdater.NetworkTablesUpdaterRegistry;

/** Parent class for for subsystems based around a single Spark Max PID loop. */
public abstract class SparkMaxPIDSubsystem extends SubsystemBase {
  SparkMaxPIDController m_mainController;
  CANSparkMax m_motor;
  CANDigitalInput lowerLimit;
  CANDigitalInput upperLimit;
  CANEncoder m_encoder;

  double m_setpointMin;
  double m_setpointMax;

  NetworkTableEntry m_setpointEntry;
  DoubleSupplier m_processVariable;

  double m_tolerance;
  /**
   * Creates a new SparkMaxPIDSubsystem, call in child class constructor
   * @param name the name of the subsystem for use in Shuffleboard
   * @param mainMotor the Spark Max to control with a PID loop
   * @param controlType the control type the subsystem uses, must be {@link com.revrobotics.ControlType kPosition or kVelocity}
   * @param tuning the initial tuning for the subsystem's PID Controller
   * @param conversionFactor the conversion factor from rotations or rotations per minute dependiong on control type
   * @param min the minimum setpoint in converted units
   * @param max the maximum setpoint in converted units
   * @param limitPolarity the polarity of attached limit switches
   */
  public SparkMaxPIDSubsystem(String name, CANSparkMax mainMotor, ControlType controlType, PIDTuningParameters tuning, double conversionFactor, double min, double max, LimitSwitchPolarity limitPolarity) {
    //set up hardware connections
    m_encoder = mainMotor.getEncoder();
    lowerLimit = mainMotor.getReverseLimitSwitch(limitPolarity);
    upperLimit = mainMotor.getForwardLimitSwitch(limitPolarity);
    m_motor = mainMotor;

    //set conversion factor depending on control type
    //set process variable accessor depending on control type
    switch (controlType){
      case kPosition: m_encoder.setPositionConversionFactor(conversionFactor);
        m_processVariable = m_encoder::getPosition;
        break;
      case kVelocity: m_encoder.setVelocityConversionFactor(conversionFactor);
        m_processVariable = m_encoder::getVelocity;
        break;
      default: throw new IllegalArgumentException("SparkMaxPIDSubsystem doesn't yet support control type "+controlType.toString());
    }
    //create teh controller object. this also sets the initial tuning
    m_mainController =new SparkMaxPIDController(mainMotor,controlType,tuning);

    //creating member copies of min and max setpoint for use outside of the constructor
    m_setpointMin = min;
    m_setpointMax = max;

    //putting information in networkTables
    //create new table for this subsystem
    NetworkTable systemTable = NetworkTableInstance.getDefault().getTable("SparkMaxPID").getSubTable(name);
    //put tuning in the table, changes from external sources will automatically update the controller
    m_mainController.addTuningToNetworkTable(systemTable.getSubTable("tuning"));
    //add entry for setpoint, external changes will update the controller
    m_setpointEntry = systemTable.getEntry("setpoint");
    m_setpointEntry.addListener(notification ->setSetpointInternal(notification.value.getDouble()), EntryListenerFlags.kUpdate);
    //add the setpoint and process variable to the table, meant to be graphed together
    NetworkTablesUpdaterRegistry registry = NetworkTablesUpdaterRegistry.getInstance();
    registry.addUpdate(m_setpointEntry, m_mainController::getSetpoint);
    registry.addUpdate(systemTable.getEntry("processVariable"), m_processVariable::getAsDouble);
  
  }
  //sets the setpoint without publishing the value to shuffleboard
  void setSetpointInternal(double newSetpoint){
    newSetpoint = MathUtil.clamp(newSetpoint, m_setpointMin, m_setpointMax);
    m_mainController.setSetpoint(newSetpoint);
  }
  /**sets the setpoint from robot code. the new value is published to shuffleboard
   * @param newSetpoint the new setpoint in the subsystem's units
   */
  public void setSetpoint(double newSetpoint){
    setSetpointInternal(newSetpoint);
    m_setpointEntry.setDouble(newSetpoint);
  }
  /**
   * sets the setpoint to the current measured process variable
   */
  public void holdCurrentPosition(){
    setSetpoint(m_processVariable.getAsDouble());
    m_mainController.reset();
  }
  /**
   * @return the current measured error between the process variable and the setpoint
   * @see {@link #onTarget()} if a numerical error isn't needed
   */
  public double getCurrentError(){
    return m_mainController.getSetpoint() - m_processVariable.getAsDouble();
  }
  /**
   * @return true if the subsystem is within it's tolerance of it's setpoint
   * @see {@link #setTolerance(double) setTolerance()} to control the width of the tolerance
   * @see {@link #getCurrentError()} if a numerical error is needed
   */
  public boolean onTarget(){
    return Math.abs(getCurrentError()) <=m_tolerance;
  }
  /**
   * sets the tolerance for determining when the subsystem is on target
   * @param tolerance the tolerance in the subsystem's units
   */
  public void setTolerance(double tolerance){
    m_tolerance = tolerance;
  }

  /** reset the encoder position if a limit switch is activated. Call in a child class method when needed. */
  void checkLimitSwitches(){
    if(lowerLimit.get()){
      m_encoder.setPosition(m_setpointMin);
    }
    if(upperLimit.get()){
      m_encoder.setPosition(m_setpointMax);
    }
  }
}
