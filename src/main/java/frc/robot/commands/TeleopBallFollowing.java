/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretCameraAim;
import frc.robot.subsystems.TurretRotator;

public class TeleopBallFollowing extends BallFollowing {
  Joystick m_joystick;
  /**
   * Creates a new TeleopBallFollowing.
   */
  public TeleopBallFollowing(Joystick joystick,Drivetrain drivetrain, TurretRotator turretrotator, TurretCameraAim limelightservo, Intake intake) {
    super(drivetrain, turretrotator, limelightservo, intake);
    m_joystick=joystick;

    // Use addRequirements() here to declare subsystem dependencies.
  }
  private final double joystickDeadband = Math.pow(.07,3);
  SlewRateLimiter joysticklimiter = new SlewRateLimiter(.7);
  @Override
  double calculatespeed() {  
    double
    y = joysticklimiter.calculate(-Math.pow(m_joystick.getY(),3));
    if( Math.abs(y)< joystickDeadband){
       y=0;
      }
      else{
       y-=Math.copySign(joystickDeadband, y);
      }
    return y;
  }
  @Override
  void noTargetDrive() {
    double x = Math.pow(m_joystick.getX(),3);
    double y = joysticklimiter.calculate(-Math.pow(m_joystick.getY(),3));
    if( Math.abs(x)<joystickDeadband){
      x=0;
      }
      else {
        x-=Math.copySign(joystickDeadband, x);
      }
    if( Math.abs(y)<joystickDeadband){
       y=0;
      }
      else{
       y-=Math.copySign(joystickDeadband, y);
      }
  m_drivetrain.ArcadeDrive(x, y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
