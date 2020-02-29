/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SetpointAccelerationLimiter;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerChannel, MotorType.kBrushless);
  private final CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax leftFollower= new CANSparkMax(Constants.leftFollowerChannel, MotorType.kBrushless);

  private final CANPIDController rightController = new CANPIDController(rightLeader);
  private final CANPIDController leftController = new CANPIDController(leftLeader);

  private final CANEncoder leftEncoder = new CANEncoder(leftLeader);
  private final CANEncoder rightEncoder = new CANEncoder(rightLeader);


  private double lP, lI, lD, lF, rP, rI, rD, rF, lSP, rSP;
  private boolean lockToLeft = false;
  private NetworkTableEntry lPEntry, lIEntry,lDEntry , lFEntry, rPEntry, rIEntry, rDEntry, rFEntry, lSPEntry, rSPEntry, lockEntry;

  private SetpointAccelerationLimiter m_setpointLimiter;
  NetworkTableEntry limitedEntry;
  
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);

    lPEntry = Shuffleboard.getTab("velocity drive tuning").add("left P", lP).getEntry();
    lIEntry = Shuffleboard.getTab("velocity drive tuning").add("left I", lI).getEntry();
    lDEntry = Shuffleboard.getTab("velocity drive tuning").add("left D", lD).getEntry();
    lFEntry = Shuffleboard.getTab("velocity drive tuning").add("left FF", lF).getEntry();
    lSPEntry = Shuffleboard.getTab("velocity drive tuning").add("left Setpoint", lSP).getEntry();

    rPEntry = Shuffleboard.getTab("velocity drive tuning").add("right P", rP).getEntry();
    rIEntry = Shuffleboard.getTab("velocity drive tuning").add("right I", rI).getEntry();
    rDEntry = Shuffleboard.getTab("velocity drive tuning").add("right D", rD).getEntry();
    rFEntry = Shuffleboard.getTab("velocity drive tuning").add("right FF", rF).getEntry();
    rSPEntry = Shuffleboard.getTab("velocity drive tuning").add("right Setpoint", rSP).getEntry();


    double velocityConversion = (7.0/12.0*Math.PI)*(1.0/8.45)*(1.0/60.0);
    double positionalConversion = (7.0/12.0*Math.PI)*(1.0/8.45);
   leftEncoder.setVelocityConversionFactor(velocityConversion);
   rightEncoder.setVelocityConversionFactor(velocityConversion);
   leftEncoder.setPositionConversionFactor(positionalConversion);
   rightEncoder.setPositionConversionFactor(positionalConversion);
   leftEncoder.setPosition(0.0);
   rightEncoder.setPosition(0.0);

    Shuffleboard.getTab("velocity drive tuning").addNumber("left pv", leftEncoder::getVelocity);
    Shuffleboard.getTab("velocity drive tuning").addNumber("right pv", rightEncoder::getVelocity);

    lockEntry = Shuffleboard.getTab("velocity drive tuning").add("lock values to left", lockToLeft).getEntry();
    Shuffleboard.getTab("velocity drive tuning").addNumber("left output", leftLeader::get);
    Shuffleboard.getTab("velocity drive tuning").addNumber("left output voltage", leftLeader::getVoltageCompensationNominalVoltage);
    initGains();

    m_setpointLimiter = new SetpointAccelerationLimiter(20, 40);
    m_setpointLimiter.setSetpoint(20);
    limitedEntry = Shuffleboard.getTab("velocity drive tuning").add("limited",0).getEntry();
  }

  void initGains(){
    lockEntry.setBoolean(true);
    lPEntry.setDouble(0.08);
    lFEntry.setDouble(0.053);
    updateGains();
  }

  void ballScan(){
    ArcadeDrive(.1, 0);
  }

  public void ArcadeDrive(double zRotation, double xSpeed){

    //frontLeft.setInverted(true);
    double leftMotorOutput;
    double rightMotorOutput;
    double maxInput = Math.copySign(Math.max(Math.abs(zRotation), Math.abs(xSpeed)), xSpeed);
    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    } 
    TankDrive(leftMotorOutput, rightMotorOutput);
  }

  public void TankDrive(double leftValue, double rightValue){
    double leftSpeedSetpoint = leftValue* Constants.maxDriveSpeed;
    double rightSpeedSetpoint = rightValue* Constants.maxDriveSpeed;
    rightController.setReference(rightValue*Constants.maxDriveSpeed, ControlType.kVelocity);
    leftController.setReference(leftValue*Constants.maxDriveSpeed, ControlType.kVelocity);

    rSPEntry.setDouble(rightSpeedSetpoint);
    lSPEntry.setDouble(leftSpeedSetpoint);

    rSP= rightSpeedSetpoint;
    lSP= leftSpeedSetpoint;
  }

  public void EmergencyStop(){
    TankDrive(0, 0);
  }

  @Override
  public void periodic() {
    updateGains();
    limitedEntry.setDouble(m_setpointLimiter.get());
    //System.out.println(leftController.getIAccum());
    // This method will be called once per scheduler run
  }
  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }
  public void resetRightEncoder(){
    rightEncoder.setPosition(0);
  }
  private void updateGains(){
    if (lockEntry.getBoolean(true) != lockToLeft){
      lockToLeft = lockEntry.getBoolean(true);
    }
    if (lockToLeft){
      lockGains();
    }
    updateUnlocked();
  }

  private void lockGains(){
    rPEntry.setDouble(lPEntry.getDouble(0));
    rIEntry.setDouble(lIEntry.getDouble(0));
    rDEntry.setDouble(lDEntry.getDouble(0));
    rFEntry.setDouble(lFEntry.getDouble(0));
    rSPEntry.setDouble(lSPEntry.getDouble(0));
  }
  private void updateUnlocked(){
    if(lP!=lPEntry.getDouble(0)){
      lP=lPEntry.getDouble(0);
      leftController.setP(lP); 
    }
    if(lI!=lIEntry.getDouble(0)){
      lI=lIEntry.getDouble(0);
      leftController.setI(lI);
    }
    if(lD!=lDEntry.getDouble(0)){
      lD=lDEntry.getDouble(0);
      leftController.setD(lD);
    }
    if(lF!=lFEntry.getDouble(0)){
      lF=lFEntry.getDouble(0);
      leftController.setFF(lF);
    }
    if(lSP!=lSPEntry.getDouble(0)){
      leftController.setIAccum(0);
      rightController.setIAccum(0);
      lSP=lSPEntry.getDouble(0);
      leftController.setReference(lSP, ControlType.kVelocity);
      leftController.setIAccum(0);
      rightController.setIAccum(0);
    }

    if(rP!=rPEntry.getDouble(0)){
      rP=rPEntry.getDouble(0);
      rightController.setP(rP);
    }
    if(rI!=rIEntry.getDouble(0)){
      rI=rIEntry.getDouble(0);
      rightController.setI(rI);
    }
    if(rD!=rDEntry.getDouble(0)){
      rD=rDEntry.getDouble(0);
      rightController.setD(rD);
    }
    if(rF!=rFEntry.getDouble(0)){
      rF=rFEntry.getDouble(0);
      rightController.setFF(rF);
    }
    if(rSP!=rSPEntry.getDouble(0)){
      leftController.setIAccum(0);
      rightController.setIAccum(0);
      rSP=rSPEntry.getDouble(0);
      rightController.setReference(rSP, ControlType.kVelocity);
    }
  }

}
