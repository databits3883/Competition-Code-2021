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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class TurretRotator extends SubsystemBase {
  private final CANSparkMax rotatorMotor = new CANSparkMax(Constants.turretRotationChannel, MotorType.kBrushless);
  private final CANEncoder encoder = new CANEncoder(rotatorMotor);
  private final CANDigitalInput zeroLimit = new CANDigitalInput(rotatorMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyOpen);
  private final CANDigitalInput upperLimit = new CANDigitalInput(rotatorMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
  private double p,i,d,ff;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,spEntry;
  private final CANPIDController controller = new CANPIDController(rotatorMotor);

  private double setpoint;

  /**
   * Creates a new TurretRotator.
   */
  public TurretRotator() {
    double gearRatio = (1.0/15.0);
    double degreeRatio = (360.0);
    double conversionFactor = gearRatio*degreeRatio;
    System.out.println(conversionFactor);
    encoder.setPositionConversionFactor(conversionFactor);
    
    pEntry = Shuffleboard.getTab("turretRotatorTuning").add("portional",p).getEntry();
    iEntry = Shuffleboard.getTab("turretRotatorTuning").add("integral",i).getEntry();
    dEntry = Shuffleboard.getTab("turretRotatorTuning").add("derivative",d).getEntry();
    ffEntry = Shuffleboard.getTab("turretRotatorTuning").add("feedForward",ff).getEntry();
    spEntry = Shuffleboard.getTab("turretRotatorTuning").add("setpoint",setpoint).getEntry();

    Shuffleboard.getTab("turretRotatorTuning").addNumber("Process Variable", () -> encoder.getPosition());
    Shuffleboard.getTab("turretRotatorTuning").addBoolean("reverse Limit",()-> upperLimit.get());
    Shuffleboard.getTab("turretRotatorTuning").addBoolean("forward Limit",()-> zeroLimit.get());

    //System.out.println(encoder.setInverted(true));
    encoder.setPosition(0);

  }
  public void setAngle(double newSetpoint){
    setpoint = MathUtil.clamp(newSetpoint, 0, Constants.maxTurretAngle);
    controller.setReference(setpoint, ControlType.kPosition);
  }
  public void changeAngle(double angleDelta){
    setAngle(setpoint + angleDelta);
  }
  
  private void updateGains(){
    if(p != pEntry.getDouble(0)){
      p = pEntry.getDouble(0);
      controller.setP(p);
    }
    if(i != iEntry.getDouble(0)){
      i = iEntry.getDouble(0);
      controller.setI(i);
    }
    if(d != dEntry.getDouble(0)){
      d = dEntry.getDouble(0);
      controller.setD(d);
    }
    if(ff != ffEntry.getDouble(0)){
      ff = ffEntry.getDouble(0); 
      controller.setFF(ff);
    }
    if(setpoint!=spEntry.getDouble(setpoint)){
      setpoint=spEntry.getDouble(setpoint);
      setAngle(setpoint);
    }
  }
  private void testLimits(){
    if(zeroLimit.get()) encoder.setPosition(0);
    if(upperLimit.get()) encoder.setPosition(Constants.maxTurretAngle);
  }
    
  
  @Override
  public void periodic() {
     // This method will be called once per scheduler run
    updateGains();
    testLimits();
    System.out.println(encoder.getInverted());
  }
}
