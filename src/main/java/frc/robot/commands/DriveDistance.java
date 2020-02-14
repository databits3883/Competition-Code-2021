/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistance extends PIDCommand {
  /**
   * Creates a new DriveDistance.
   * @param distance distance to drive in feet
   * @param drivetrain is the drivetrain
   */
  public DriveDistance(double distance,Drivetrain drivetrain) {

    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> drivetrain.getLeftEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.ArcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
