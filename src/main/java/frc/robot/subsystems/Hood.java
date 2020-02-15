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
import frc.robot.util.SetpointVelocityLimiter;

public class Hood extends SubsystemBase {
  /**
  * Creates a new Hood.
  */  
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.turretHoodChannel, MotorType.kBrushless);
  private final CANDigitalInput lowerLimit = new CANDigitalInput(hoodMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyClosed);
  private final CANDigitalInput upperLimit = new CANDigitalInput(hoodMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyClosed);
  private final CANPIDController controller = new CANPIDController(hoodMotor);
  private final CANEncoder encoder = new CANEncoder(hoodMotor);
  private double p,i,d,ff;
  private double angle;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,spEntry;

  private SetpointVelocityLimiter velocityLimiter = new SetpointVelocityLimiter(Constants.maxHoodVelocity);
 
  public Hood() {

    encoder.setPositionConversionFactor(16.0*360.0/264.0/50.0);

    pEntry = Shuffleboard.getTab("hoodTuning").add("portional",p).getEntry();
    iEntry = Shuffleboard.getTab("hoodTuning").add("integral",i).getEntry();
    dEntry = Shuffleboard.getTab("hoodTuning").add("derivative",d).getEntry();
    ffEntry = Shuffleboard.getTab("hoodTuning").add("feedForward",ff).getEntry();
    spEntry = Shuffleboard.getTab("hoodTuning").add("setpoint",angle).getEntry();

    Shuffleboard.getTab("hoodTuning").addBoolean("lowerSwitch", ()->lowerLimit.get());
    Shuffleboard.getTab("hoodTuning").addBoolean("upperSwitch", ()->upperLimit.get());
    Shuffleboard.getTab("hoodTuning").addNumber("current angle", ()->encoder.getPosition());
  }

  @Override
  public void periodic() {
    updateGains();
    checkEndpoints();
    controller.setReference(velocityLimiter.get(), ControlType.kPosition);
    // This method will be called once per scheduler run
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
    if(angle!=spEntry.getDouble(angle)){
      angle = spEntry.getDouble(angle);
      setAngle(angle);

    }
   
  }
  private void checkEndpoints(){
    if(lowerLimit.get()){
      encoder.setPosition(Constants.minimumHoodAngle);
    }
    if(upperLimit.get()){
      encoder.setPosition(Constants.maximumHoodAngle);
    }
  }
  public void setAngle(double newAngle){
    angle = MathUtil.clamp(newAngle, Constants.minimumHoodAngle, Constants.maximumHoodAngle);
    velocityLimiter.setTarget(angle);
  }
  public void changeAngle(double angleDelta){
    setAngle(angle + angleDelta);
  }
}
