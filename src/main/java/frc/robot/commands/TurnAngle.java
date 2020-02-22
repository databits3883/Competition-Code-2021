/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Variables;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnAngle extends PIDCommand {
  Drivetrain m_drivetrain;
  /**
   * Creates a new DriveDistance.
   * @param angle angle to drive in degrees
   * @param drivetrain is the drivetrain
   */
  public TurnAngle(double angle ,Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> Variables.getInstance().getGyroAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drivetrain.ArcadeDrive(output,0 );
          // Use the output here
        });
        addRequirements(drivetrain);
    Shuffleboard.getTab("Turning driving tuning").add(getController());
    Shuffleboard.getTab("Turning driving tuning").addNumber("current distance", ()->drivetrain.getLeftEncoder());
    Shuffleboard.getTab("Turning driving tuning").addNumber("target distance", ()->angle);
    m_drivetrain = drivetrain;
  }
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
