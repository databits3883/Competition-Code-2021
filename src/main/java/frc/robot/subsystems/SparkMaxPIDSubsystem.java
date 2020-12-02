/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;

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

    //putting information in shuffleboard
    //create the layout for containing everything in the subsystem
    ShuffleboardLayout container = Shuffleboard.getTab(name).getLayout(name,BuiltInLayouts.kList).withSize(2, 5).withPosition(0, 0).withProperties(Map.of("Label position","TOP"));
    ShuffleboardLayout tuningLayout = container.getLayout("tuning","Grid Layout").withSize(2, 2).withProperties(SparkMaxPIDController.tuningDisplayMap);
    //puts tuning on shuffleboard, as a side effect this automatically updates tuning when shuffleboard changes
    m_mainController.addTuningToShuffleboard(tuningLayout);
    //add a graph with the setpoint and the current value
    container.addDoubleArray("Process Variable vs Setpoint", ()->(new double[] {m_processVariable.getAsDouble(),m_mainController.getSetpoint()}))
      .withWidget(BuiltInWidgets.kGraph);
    //adds a slider for setting the setpoint on shuffleboard
    m_setpointEntry = container.add("setpoint", m_mainController.getSetpoint())
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("Min",m_setpointMin,"Max",m_setpointMax))
      .getEntry();
    m_setpointEntry.addListener(notification ->setSetpointInternal(notification.value.getDouble()), EntryListenerFlags.kUpdate);
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
