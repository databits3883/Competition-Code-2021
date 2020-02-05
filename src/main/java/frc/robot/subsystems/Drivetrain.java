/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax frontRight = new CANSparkMax(Constants.frontRightChannel, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(Constants.rearRightChannel, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(Constants.frontLeftChannel, MotorType.kBrushless);
  private final CANSparkMax rearLeft= new CANSparkMax(Constants.rearLeftChannel, MotorType.kBrushless);

  private final CANPIDController rightController = new CANPIDController(frontRight);
  private final CANPIDController leftController = new CANPIDController(frontLeft);
  
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    rearRight.follow(frontRight);
    rearLeft.follow(frontLeft);
  }

  public void ArcadeDrive(double zRotation, double xSpeed){

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
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
    rightController.setReference(rightValue*Constants.maxDriveSpeed, ControlType.kSmartVelocity);
    leftController.setReference(leftValue*Constants.maxDriveSpeed, ControlType.kSmartVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
