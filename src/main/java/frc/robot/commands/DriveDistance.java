/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistance extends ProfiledPIDCommand {
  Drivetrain m_drivetrain;
  /**
   * Creates a new DriveDistance.
   */
  public DriveDistance(double distance,Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.2, 0, 0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(15, 7)),
        // This should return the measurement
        () -> drivetrain.getRightEncoder(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(distance,0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.ArcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    Shuffleboard.getTab("drive distance tuning").add("controller",getController());
    Shuffleboard.getTab("drive distance tuning").addNumber("setpoint",()->getController().getSetpoint().position);
    Shuffleboard.getTab("drive distance tuning").addNumber("pv", ()->drivetrain.getRightEncoder());
    getController().setTolerance(1.0/12.0, (2.0/12.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
  @Override
  public void initialize() {
    super.initialize();
    m_drivetrain.resetRightEncoder();
    getController().reset(0);
  }
  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    System.out.println("distance ended");
  }
}
