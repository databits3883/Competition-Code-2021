/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.SetpointVelocityLimiter;
import frc.robot.Variables;

public class TurretLauncher extends SubsystemBase {
  private final CANSparkMax leader = new CANSparkMax(Constants.launcherLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.launcherFollowerChannel, MotorType.kBrushless);

  private final CANEncoder encoder = new CANEncoder(leader);
  private final CANPIDController controller = new CANPIDController(leader);

  private double p,i,d,ff,speed;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,speedEntry;
  NetworkTableEntry limitedentry;

  public SetpointVelocityLimiter m_setpointLimiter = new SetpointVelocityLimiter(25);
  /**
   * Creates a new Launcher.
   */
  public TurretLauncher() {
    follower.follow(leader,true);
    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);
    encoder.setVelocityConversionFactor((4.0/12.0*Math.PI)*(1.0/60.0));

    pEntry = Shuffleboard.getTab("LaunchingTuning").add("portional",p).getEntry();
    iEntry = Shuffleboard.getTab("LaunchingTuning").add("integral",i).getEntry();
    dEntry = Shuffleboard.getTab("LaunchingTuning").add("derivative",d).getEntry();
    ffEntry = Shuffleboard.getTab("LaunchingTuning").add("feedForward",ff).getEntry();

    Shuffleboard.getTab("LaunchingTuning").addNumber("pdp voltage", ()->Variables.getInstance().getPDPCurrent(3));
    Shuffleboard.getTab("LaunchingTuning").addNumber("pv", encoder::getVelocity);
    Shuffleboard.getTab("LaunchingTuning").addNumber("pv - rpm", ()->encoder.getVelocity()/encoder.getVelocityConversionFactor());
    speedEntry = Shuffleboard.getTab("LaunchingTuning").add("speed",speed).getEntry();

    m_setpointLimiter.setTarget(0);

    initGains();
  }

  void initGains(){
    pEntry.setDouble(0.03);
    ffEntry.setDouble(0.005);
    dEntry.setDouble(0.1);
    updateGains();
  }

  /**
   * Sets the speed of the Launcher wheel
   * @param speed the target speed of the wheel in inches per second
   */
  public void setSpeed(double newSpeed){
    m_setpointLimiter.setTarget(newSpeed);
    speed = newSpeed;
    speedEntry.setDouble(speed);
  }
  public void changeSpeed(double angleDelta){
    setSpeed(speed + angleDelta);
  }

  public boolean atSpeed(){
    return Math.abs(speed - encoder.getVelocity()) <1;
  }
  
  
  boolean lastTrigger = false;
  double triggerAmps = 9;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateGains();
    double newSp = m_setpointLimiter.get();
    controller.setReference(newSp, ControlType.kVelocity);
    Variables.getInstance().setShooterEnabled((encoder.getVelocity() <=-Constants.minimumShootSpeed));
    Variables.getInstance().setShooterAtSpeed(encoder.getVelocity()-speed<=2);
   

    double amps = Variables.getInstance().getPDPCurrent(3);
    boolean trigger = amps>=triggerAmps;
   
    if(trigger && !lastTrigger){
      Variables.getInstance().subtractPowerCell();
    }
    lastTrigger = trigger;
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
    if(speed != speedEntry.getDouble(0)){
      speed = speedEntry.getDouble(0); 
      setSpeed(speed);
    }
  }
}
