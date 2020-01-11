/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax frontRight = new CANSparkMax(Constants.frontRightChannel, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(Constants.rearRightChannel, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(Constants.frontLeftChannel, MotorType.kBrushless);
  private final CANSparkMax rearLeft= new CANSparkMax(Constants.rearLeftChannel, MotorType.kBrushless);

  private final SpeedControllerGroup left = new SpeedControllerGroup(frontRight, rearRight);
  private final SpeedControllerGroup right = new SpeedControllerGroup(frontLeft, rearLeft);
 
  private final DifferentialDrive drive = new DifferentialDrive(left, right);
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {

  }

  public void ArcadeDrive(double rotation, double speed){
    drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
