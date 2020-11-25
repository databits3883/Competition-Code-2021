/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerChannel, MotorType.kBrushless);
  private final CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax leftFollower= new CANSparkMax(Constants.leftFollowerChannel, MotorType.kBrushless);

  private final SparkMaxPIDController m_rightController;
  private final SparkMaxPIDController m_leftController;

  private final CANEncoder leftEncoder = new CANEncoder(leftLeader);
  private final CANEncoder rightEncoder = new CANEncoder(rightLeader);
  
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


    double velocityConversion = (7.0/12.0*Math.PI)*(1.0/8.45)*(1.0/60.0);
    double positionalConversion = (7.0/12.0*Math.PI)*(1.0/8.45);
    leftEncoder.setVelocityConversionFactor(velocityConversion);
    rightEncoder.setVelocityConversionFactor(velocityConversion);
    leftEncoder.setPositionConversionFactor(positionalConversion);
    rightEncoder.setPositionConversionFactor(positionalConversion);
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    PIDTuningParameters rightTuning = new PIDTuningParameters(0.08, 0, 0,0.053);
    PIDTuningParameters leftTuning = new PIDTuningParameters(0.08, 0, 0,0.053);

    m_rightController = new SparkMaxPIDController(rightLeader, ControlType.kVelocity, rightTuning);
    m_leftController = new SparkMaxPIDController(leftLeader, ControlType.kVelocity, leftTuning);

    ShuffleboardTab tab = Shuffleboard.getTab("Velocity Drive");
    m_rightController.addTuningToShuffleboard(tab.getLayout("right tuning", "Grid Layout"));
    m_leftController.addTuningToShuffleboard(tab.getLayout("left tuning", "Grid Layout"));

    tab.addNumber("left pv", leftEncoder::getVelocity);
    tab.addNumber("right pv", rightEncoder::getVelocity);

    tab.addNumber("left output", leftLeader::get);

    tab.addDoubleArray("right setpoint vs pv", ()-> (new double[] {rightEncoder.getVelocity(), m_leftController.getSetpoint()})).withWidget(BuiltInWidgets.kGraph);
    tab.addDoubleArray("left setpoint vs pv", ()-> (new double[] {leftEncoder.getVelocity(), m_leftController.getSetpoint()})).withWidget(BuiltInWidgets.kGraph);
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
    m_rightController.setSetpoint(rightValue*Constants.maxDriveSpeed);
    m_leftController.setSetpoint(leftValue*Constants.maxDriveSpeed);
  }

  public void EmergencyStop(){
    TankDrive(0, 0);
  }

  @Override
  public void periodic() {
    //System.out.println(leftController.getIAccum());
    // This method will be called once per scheduler run
  }
  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }
  public void resetRightEncoder(){
    rightEncoder.setPosition(0);
  }
}
