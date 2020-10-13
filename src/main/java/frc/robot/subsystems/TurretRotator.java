/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.SetpointVelocityLimiter;
import frc.robot.util.SparkMaxPIDController;

public class TurretRotator extends SubsystemBase {
  private final CANSparkMax rotatorMotor = new CANSparkMax(Constants.turretRotationChannel, MotorType.kBrushless);
  private final CANEncoder encoder = new CANEncoder(rotatorMotor);
  private final CANDigitalInput forwardLimit = new CANDigitalInput(rotatorMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
  private final CANDigitalInput reverseLimit = new CANDigitalInput(rotatorMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
  private double p,i,d,ff;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,spEntry,outputEntry;
  private final CANPIDController controller = new CANPIDController(rotatorMotor);
  private final SetpointVelocityLimiter velocityLimiter;

  private double setpoint;

  /**
   * Creates a new TurretRotator.
   */
  public TurretRotator() {
    double gearRatio = (1.0/15.0);
    double degreeRatio = (360.0);
    double conversionFactor = gearRatio*degreeRatio;
    
    setpoint=encoder.getPosition();
    controller.setReference(setpoint, ControlType.kPosition);

    encoder.setPositionConversionFactor(conversionFactor);
    velocityLimiter = new SetpointVelocityLimiter(Constants.maxTurretVelocity);
   
    SparkMaxPIDController controllerContainer = new SparkMaxPIDController(rotatorMotor);
    Shuffleboard.getTab("turretRotatorTuning").add(controllerContainer);
    controllerContainer.setP(0.04);
    controllerContainer.setI(0);
    controllerContainer.setD(0);
    
    
    spEntry.setDouble(setpoint);

    Shuffleboard.getTab("turretRotatorTuning").addNumber("Process Variable", () -> encoder.getPosition());
    Shuffleboard.getTab("turretRotatorTuning").addBoolean("reverse Limit",()-> reverseLimit.get());
    Shuffleboard.getTab("turretRotatorTuning").addBoolean("forward Limit",()-> forwardLimit.get());
    Shuffleboard.getTab("turretRotatorTuning").addNumber("output",()->rotatorMotor.get());
    setCurrentPosition();
    
  }
  
  public void setAngle(double newSetpoint){
    
    setpoint = MathUtil.clamp(newSetpoint, 0, Constants.maxTurretAngle);
    spEntry.setDouble(setpoint);
    velocityLimiter.setTarget(setpoint);
  }
  public void setAngleStep(double newSetpoint){
    controller.setIAccum(0);
    controller.getIAccum();
    setAngle(newSetpoint);
  }
  public void changeAngle(double angleDelta){
    //System.out.println(angleDelta);
    setAngle(setpoint+ angleDelta);
  }

  public void setCurrentPosition(){
    controller.setIAccum(0);
    
    setpoint=encoder.getPosition();
    velocityLimiter.setWithoutRamp(setpoint);
    spEntry.setDouble(setpoint);
  }
  public double getCurrentAngle(){
    return encoder.getPosition();
  }
  public double getCurrentError(){
    return velocityLimiter.getCurrentTarget() - getCurrentAngle();
  }
  public boolean atTarget(){
    return Math.abs(setpoint - encoder.getPosition())<1.5;
  }
  private void testLimits(){
    if(forwardLimit.get()) encoder.setPosition(Constants.maxTurretAngle);
    if(reverseLimit.get()) encoder.setPosition(0);
  }
    
  
  @Override
  public void periodic() {
     // This method will be called once per scheduler run

    if(DriverStation.getInstance().isDisabled()) testLimits();
   
    controller.setReference(velocityLimiter.get(), ControlType.kPosition);
    if(encoder.getVelocity()>900&&false){
      System.out.println("Turret moving too fast! "+encoder.getVelocity()+" rpm");
      System.exit(1);

    }
    
  }
}
