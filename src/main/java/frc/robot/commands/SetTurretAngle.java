/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretRotator;

public class SetTurretAngle extends CommandBase {
  TurretRotator m_turretRotator;
  double targetAngle;
  /**
   * Creates a new SetTurretAngle.
   */
  public SetTurretAngle(double angle, TurretRotator rotator) {
    addRequirements(rotator);
    m_turretRotator = rotator;
    targetAngle = angle;
    //Shuffleboard.getTab("commandDebugging").add(this.getName() + " angle: " + angle,this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turretRotator.setAngle(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("turret angle set");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turretRotator.atTarget();
  }
}
