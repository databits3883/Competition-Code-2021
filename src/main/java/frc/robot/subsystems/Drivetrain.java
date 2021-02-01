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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.PIDTuningParameters;
import frc.robot.util.SparkMaxPIDController;
import frc.robot.util.NetworkTablesUpdater.NetworkTablesUpdaterRegistry;
import frc.robot.util.Odometry;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.util.SetpointAccelerationLimiter;


public class Drivetrain extends SubsystemBase {
  private final CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerChannel, MotorType.kBrushless);
  private final CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderChannel, MotorType.kBrushless);
  private final CANSparkMax leftFollower= new CANSparkMax(Constants.leftFollowerChannel, MotorType.kBrushless);

  private final SparkMaxPIDController m_rightController;
  private final SparkMaxPIDController m_leftController;

  private final CANEncoder leftEncoder = new CANEncoder(leftLeader);
  private final CANEncoder rightEncoder = new CANEncoder(rightLeader);

  private double v;
  private double lastPosition;
  private double lastPitch = 0;
  private double currentAngle;
  private double[] robotCoordinates = new double[2];

  double resetableEncoderLeft;
  double resetableEncoderRight;
  double lastDistanceLeft;
  double lastDistanceRight;
  double deltaLeft;
  double deltaRight;

  private Pose2d robotPosition;
  private Pose2d zeroedPose= new Pose2d();
  private Rotation2d zeroedRot = new Rotation2d(0);
  private DifferentialDriveOdometry robotOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0));
  private Rotation2d robotRotation;
  private Translation2d robotTranslation;
  Drivetrain m_drivetrain;
  Odometry m_odometry;


  private double lP, lI, lD, lF, rP, rI, rD, rF, lSP, rSP;
  private boolean lockToLeft = false;
  private NetworkTableEntry lPEntry, lIEntry,lDEntry , lFEntry, rPEntry, rIEntry, rDEntry, rFEntry, lSPEntry, rSPEntry, lockEntry;

  private SetpointAccelerationLimiter m_setpointLimiter;
  NetworkTableEntry limitedEntry;

  NetworkTableEntry robotCoordinatesEntry = NetworkTableInstance.getDefault().getTable("Robot Coordinates").getEntry("Coordinates Array");
  NetworkTableEntry leftAndRightEncodersResetable = NetworkTableInstance.getDefault().getTable("Resetable Encoders").getEntry("Left and Right");
  public double[] resetableEncoderVals = new double[2];
  public void ResetOdometry(){
    robotOdometry.resetPosition(zeroedPose, zeroedRot);
    Variables.getInstance().resetNavx();
    resetableEncoderRight = 0;
    resetableEncoderLeft = 0;
  }

  
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(Odometry odometry) {
    m_odometry = odometry;
    leftLeader.setInverted(false);
    rightLeader.setInverted(true);

    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);

    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);


    double velocityConversion = (Constants.wheelDiameter*Math.PI)*(1.0/Constants.driveTrainGearingRatio)*(1.0/60.0);
    double positionalConversion = (Constants.wheelDiameter*Math.PI)*(1.0/Constants.driveTrainGearingRatio);
    leftEncoder.setVelocityConversionFactor(velocityConversion);
    rightEncoder.setVelocityConversionFactor(velocityConversion);
    leftEncoder.setPositionConversionFactor(positionalConversion);
    rightEncoder.setPositionConversionFactor(positionalConversion);
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    //robotPosition = new Pose2d(0,0,new Rotation2d(0));

    PIDTuningParameters rightTuning = new PIDTuningParameters(0.24, 0, 0,0.159);
    PIDTuningParameters leftTuning = new PIDTuningParameters(0.24, 0, 0,0.159);

    m_rightController = new SparkMaxPIDController(rightLeader, ControlType.kVelocity, rightTuning);
    m_leftController = new SparkMaxPIDController(leftLeader, ControlType.kVelocity, leftTuning);

    NetworkTable pidTable = NetworkTableInstance.getDefault().getTable("SparkMaxPID").getSubTable("VelocityDrive");
    m_rightController.addTuningToNetworkTable(pidTable.getSubTable("rightTuning"));
    m_leftController.addTuningToNetworkTable(pidTable.getSubTable("leftTuning"));


    NetworkTablesUpdaterRegistry registry = NetworkTablesUpdaterRegistry.getInstance();

    NetworkTable processTable = pidTable.getSubTable("processVariables");
    registry.addUpdate(processTable.getEntry("left"), leftEncoder::getVelocity);
    registry.addUpdate(processTable.getEntry("right"), rightEncoder::getVelocity);

    NetworkTable outputTable = pidTable.getSubTable("output");
    registry.addUpdate(outputTable.getEntry("left"), leftLeader::get);
    registry.addUpdate(outputTable.getEntry("right"), rightLeader::get);

    NetworkTable setpointTable = pidTable.getSubTable("setpoints");
    registry.addUpdate(setpointTable.getEntry("left"),m_leftController::getSetpoint);
    registry.addUpdate(setpointTable.getEntry("right"), m_rightController::getSetpoint);
  }

  public void ArcadeDrive(double zRotation, double xSpeed){
    
    //telemetry[1] = Variables.getInstance().G
    telemetry[4] = xSpeed;


    //frontLeft.setInverted(true);
    double leftMotorOutput;
    double rightMotorOutput;
    double maxInput = Math.copySign(Math.max(Math.abs(zRotation), Math.abs(xSpeed)), xSpeed);
    double LeftDistanceMeters;
    double RightDistanceMeters;
    int slow_print;
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
    double tippingOffset = getTippingOffset();
    leftValue=MathUtil.clamp(leftValue+tippingOffset, -1, 1);
    rightValue=MathUtil.clamp(rightValue+tippingOffset, -1, 1);

    m_rightController.setSetpoint(rightValue*Constants.maxDriveSpeed);
    m_leftController.setSetpoint(leftValue*Constants.maxDriveSpeed);
  }


  public void absoluteTankDriveMeters(double leftVelocity, double rightVelocity){
    m_rightController.setSetpoint(rightVelocity);
    m_leftController.setSetpoint(leftVelocity);
}
   double getTippingOffset(){
    double tip = Variables.getInstance().getGyroPitch(); //positive pitch is nose-up: negative offset to fix
    double tipMagnitude = Math.abs(tip);
    if(tipMagnitude<=5){
      return 0;
    } 
    else{
      return -1*Math.copySign(0.1*10*Math.pow((tipMagnitude-5)/10.0, 2), tip);
    }

  }

  public void EmergencyStop(){
    TankDrive(0, 0);
  }
  public void DistanceReset(){
    resetableEncoderLeft = 0;
    resetableEncoderRight = 0;
  }



  

  @Override
  public void periodic() {
    
    //System.out.println(leftController.getIAccum());
    // This method will be called once per scheduler run

    //if(m_odometry.tv.getBoolean(false) && !m_odometry.wasValid){
    //  m_odometry.updateFromReflectors();
     // m_odometry.wasValid = true;
    //}
    
    robotRotation = Rotation2d.fromDegrees(-1.0*currentAngle);
    

    //robotPosition =  robotOdometry.getPoseMeters();
    //robotTranslation = robotPosition.getTranslation();
    robotPosition = robotOdometry.update(robotRotation, resetableEncoderLeft, resetableEncoderRight);
    currentAngle = Variables.getInstance().getGyroAngle();
    
    //System.out.println("x Translation" + robotTranslation.getX());
    //System.out.println("y Translation" + robotTranslation.getY());
    resetableEncoderLeft = resetableEncoderLeft + deltaLeft;
    resetableEncoderRight = resetableEncoderRight + deltaRight;

    deltaLeft = GetLeftEncoder() - lastDistanceLeft;
    deltaRight = getRightEncoder() - lastDistanceRight;

    lastDistanceLeft = GetLeftEncoder();
    lastDistanceRight = getRightEncoder();
    robotCoordinates[0] = robotPosition.getTranslation().getX();
    robotCoordinates[1] = robotPosition.getTranslation().getY();
    robotCoordinatesEntry.setDoubleArray(robotCoordinates);
    leftAndRightEncodersResetable.setDoubleArray(resetableEncoderVals);
    //System.out.println("Resetable Left"+ resetableEncoderLeft);
    //System.out.println("Resetable Right"+resetableEncoderRight);
    resetableEncoderVals[0] = resetableEncoderLeft;
    resetableEncoderVals[1] = resetableEncoderRight;
    

    lastPosition = GetEncodersTotal();
    //robotCoordinatesInstance.

    //telemetryEntry.setDefaultDoubleArray(telemetry);
    
    
    lastPitch = Variables.getInstance().getGyroPitch();
    lastPosition = GetEncodersTotal();
    
    
  }
  public double GetPitchRate(double pitch, double lastPitch){
      return (pitch - lastPitch) * 500;

  }
  public double Test(){
    return 3;

  }
  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }
  public double GetLeftEncoder(){
    return leftEncoder.getPosition();
  }
  public double GetEncodersTotal(){
    return ((getRightEncoder()+GetLeftEncoder())/2);
  }
  public double GetSpeedInMetersPerSecond(double lastPosition){
    return (GetEncodersTotal() - lastPosition)*500;
  }
  public void resetRightEncoder(){
    rightEncoder.setPosition(0);
  }


  public Pose2d getRobotPose(){
    return robotPosition;
  }

  

}

