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

/**Aiming hood subsystem. Manages the hood moving motor.*/
public class Hood extends SubsystemBase {
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.turretHoodChannel, MotorType.kBrushless);
  private final CANDigitalInput lowerLimit = new CANDigitalInput(hoodMotor, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyClosed);
  private final CANDigitalInput upperLimit = new CANDigitalInput(hoodMotor, LimitSwitch.kForward, LimitSwitchPolarity.kNormallyClosed);
  private final CANPIDController controller = new CANPIDController(hoodMotor);
  private final CANEncoder encoder = new CANEncoder(hoodMotor);
  private double p,i,d,ff;
  private double angle;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,spEntry;

  private SetpointVelocityLimiter velocityLimiter = new SetpointVelocityLimiter(Constants.maxHoodVelocity);
   /**
  * Creates a new Hood subsystem instance.
  */  
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
    initGains();
  }

  /**Sets default values for PID gains to network tables, then updates controller gains from network tables.*/
  void initGains(){
    pEntry.setDouble(0.02);
    updateGains();
  }

  /**Is the hood at the set target angle? 
   * @return True if current hood angle is within tolerance of the set target angle.
  */
  public boolean atAngle(){
    return Math.abs(angle - encoder.getPosition())<=1.5;
  }

  /**Updates PID gains from shuffleboard. Checks end switches for the hood. Updates the spark max's setpoint to velocity limit.
   * <p>Should be called in the Robot periodic method</p>
  */
  @Override
  public void periodic() {
    updateGains();
    checkEndpoints();
    controller.setReference(velocityLimiter.get(), ControlType.kPosition);
    // This method will be called once per scheduler run
  }
  /**Updates spark max PID gains from network tables */
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
  /**Checks if limit switches are pressed and corrects position if they are.*/
  private void checkEndpoints(){
    if(lowerLimit.get()){
      encoder.setPosition(Constants.minimumHoodAngle);
    }
    if(upperLimit.get()){
      encoder.setPosition(Constants.maximumHoodAngle);
    }
  }
  /**Sets absolute angle setpoint.
   * @param newAngle angle to set hood to.
  */
  public void setAngle(double newAngle){
    angle = MathUtil.clamp(newAngle, Constants.minimumHoodAngle, Constants.maximumHoodAngle);
    spEntry.setDouble(angle);
    velocityLimiter.setTarget(angle);
  }
  /**Sets angle setpoint relative to current setpoint.
   * @param angleDelta angle to add to current setpoint.
   */
  public void changeAngle(double angleDelta){
    setAngle(angle + angleDelta);
  }
  
/**sets setpoint to current measure position
 * <p> meant to be run while disabled</p>
 */
  public void setCurrentPosition(){

    angle=encoder.getPosition();
    velocityLimiter.setWithoutRamp(angle);
    spEntry.setDouble(angle);
  }
  //TODO: find where this method is used. Maybe a command written by jacob?
  /**Gets current angle adjusted to use in <b>??</b> */
  public double LaunchAngle(){
    return (57.3-(encoder.getPosition()-17));
  }
}
