/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.util.PIDTuningParameters;
import frc.robot.Variables;

public class TurretLauncher extends ProfiledSparkMaxPIDSubsystem{
  private static final double conversionFactor = (4.0/12.0*Math.PI)*(1.0/60.0);

  private final CANSparkMax m_follower;
  /**
   * Creates a new Launcher.
   */
  public TurretLauncher() {
    super("Turret Launcher",new CANSparkMax(Constants.launcherLeaderChannel, MotorType.kBrushless),ControlType.kVelocity,new PIDTuningParameters(0.03,0,0.1,0.005),conversionFactor,
    -200,0,
    25,LimitSwitchPolarity.kNormallyOpen);
    m_follower = new CANSparkMax(Constants.launcherFollowerChannel,MotorType.kBrushless);

    m_follower.follow(m_motor,true);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_follower.setIdleMode(IdleMode.kCoast);
    
    Shuffleboard.getTab("LaunchingTuning").addNumber("pdp voltage", ()->Variables.getInstance().getPDPCurrent(3));
    setTolerance(2);
  }
  /**
   * Sets the speed of the Launcher wheel
   * @param speed the target speed of the wheel in inches per second
   */
  public void setSpeed(double newSpeed){
    setSetpoint(newSpeed);
  }
  public void changeSpeed(double angleDelta){
    setSetpoint(m_mainController.getSetpoint()+angleDelta);
  }

  public boolean atSpeed(){
    return onTarget();
  }

  public double getSpeed(){
    return m_mainController.getSetpoint();
  }
  
  
  boolean lastTrigger = false;
  double triggerAmps = 9;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    Variables.getInstance().setShooterEnabled((m_processVariable.getAsDouble() <=-Constants.minimumShootSpeed));
    Variables.getInstance().setShooterAtSpeed(atSpeed());
   

    double amps = Variables.getInstance().getPDPCurrent(3);
    boolean trigger = amps>=triggerAmps;
   
    if(trigger && !lastTrigger){
      Variables.getInstance().subtractPowerCell();
    }
    lastTrigger = trigger;
  }
}
