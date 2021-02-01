/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.subsystems.TurretCameraAim;

public class Odometry  {
  double v;
  double lastPosition;
  double currentAngle;
  double resetableEncoderLeft;
  double resetableEncoderRight;
  double lastDistanceLeft;
  double lastDistanceRight;
  Pose2d robotPosition;
  double deltaDistanceLeft;
  double deltaDistanceRight;
  DifferentialDriveOdometry robotOdometry;
  Rotation2d robotRotation;
  Translation2d robotTranslation;
  
  TurretCameraAim m_limeLightServo;
  boolean wasValid = false;
  NetworkTableEntry tv;

  public void DistanceReset(){
    resetableEncoderLeft = 0;
    resetableEncoderRight = 0;
  }

  public void updateFromReflectors(){
    //NetworkTableEntry tx;
    //NetworkTableEntry ty;
    NetworkTableEntry thor;
    
    NetworkTableEntry Pipeline;
    Pose2d updatePose;
    Translation2d updateTranslation;
    //tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    //ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor"); //percentage of pixels taken up by the target.
    
    Pipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");

    double areaAngle = 90 - ((0.5 * thor.getDouble(0)) * 59.6); //the angle that the thor line (always parallel to the face of the camera) makes with the line that connects the camera and the thor plane.
    // As a right triangle, the thor line would be a side that forms a right angle with the line that goes from the camera to where its looking. Its length is determined by thor. The last side is the one mentioned above.
    // The angle that this side makes with the thor side is areaAngle. This angle is useful later because it is supplementary with an angle we need for another triangle.

    double b = (Math.sin((0.5 * thor.getDouble(0) * 59.6)) *
    Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()))/Math.sin(areaAngle); //the actual length of thor at the center of the target. The thor line.

    double c = 0.498475; //half of the length of the target. One of the sides on the other triangle.

    double bAngle = Math.asin((b*(Math.sin(180-areaAngle)))/c); // On the new triangle, one side is half of the power port target, one side is b, or the thor line, and the last connects these. Call this a.
    //This angle is where a meets the power port target line, or c.

    double cAngle = 180 - areaAngle; // where a meets b. Supplementary with areaAngle.

    double aAngle = 180 - (bAngle+cAngle); // the last angle in this triangle

    double angleFromTarget = aAngle + 90; //if code doesnt work, try using this rather than aAngle. This is the angle of the robot's position relative to the target.
    if (robotTranslation.getY() < Constants.fieldLengthMeters/3){ // if the robot is on the lower end of the field.
      if(robotTranslation.getX() > Constants.powerPortToWallLenght){ // if the robot is on the right side of the power port relative to the power port.
        updateTranslation = new Translation2d(Math.sin(aAngle) *
        Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()) + Constants.powerPortToWallLenght,
        Math.cos(aAngle) * Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()));
        updatePose = new Pose2d(updateTranslation, robotRotation);
        robotOdometry.resetPosition(updatePose, robotRotation);
      }
      else if(robotTranslation.getX() < Constants.powerPortToWallLenght){ //if the robot is on the left side of the power port relative to the power port.
        updateTranslation = new Translation2d(Constants.powerPortToWallLenght-(Math.sin(aAngle) *
        Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle())) ,
        Math.cos(aAngle) * Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()));
        updatePose = new Pose2d(updateTranslation, robotRotation);
        robotOdometry.resetPosition(updatePose, robotRotation);
      }
    }
    else if (robotTranslation.getY()>Constants.fieldLengthMeters*(2.0/3.0)){
      if(robotTranslation.getX()<(Constants.fieldWidthMeters-Constants.powerPortToWallLenght)){
        updateTranslation = new Translation2d(Constants.fieldWidthMeters-(Math.sin(aAngle) *
        Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()) + Constants.powerPortToWallLenght),
        Constants.fieldLengthMeters-(Math.cos(aAngle) * Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle())));
        updatePose = new Pose2d(updateTranslation, robotRotation);
        robotOdometry.resetPosition(updatePose, robotRotation);
      }
      else if(robotTranslation.getX() > Constants.fieldWidthMeters - Constants.powerPortToWallLenght){
        updateTranslation = new Translation2d(Constants.fieldWidthMeters-(Constants.powerPortToWallLenght-(Math.sin(aAngle) *
        Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle()))) ,
        Constants.fieldLengthMeters - (Math.cos(aAngle) * Variables.getInstance().DistanceFromPowerPortMeters(m_limeLightServo.getAngle())));
        updatePose = new Pose2d(updateTranslation, robotRotation);
        robotOdometry.resetPosition(updatePose, robotRotation);
      }
    }
    


  }
  
  /**
   * Creates a new Odometry.
   */
  public Odometry() {
    
    
    
    
  
    
  
    
    
    
    
    

  }

  
}
