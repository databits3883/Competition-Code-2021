// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends InstantCommand {
  Drivetrain m_drivetrain;
  double m_x;
  double m_y;
  double m_angle;
  /** Creates a new ResetOdometry. */
  
  public ResetOdometry(Drivetrain drivetrain, double xPosition, double yPosition, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = drivetrain;
    m_x = xPosition;
    m_y = yPosition;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPose(m_x, m_y, m_angle);
  }
}

  // Called every time the scheduler runs while the command is scheduled.
  