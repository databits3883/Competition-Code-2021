/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopBallFollowing extends BallFollowing {
  /**
   * Creates a new TeleopBallFollowing.
   */
  public TeleopBallFollowing(DoubleSupplier yAxis, Drivetrain drivetrain, TurretRotator turretrotator, LimelightServo limelightservo, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.

  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
    m_drivetrain.ArcadeDrive(-turnpid.calculate(tx.getDouble(0), 0), yAxis);
    
    verticalAim();
    
    

  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
