/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveDistance extends PIDCommand {
  Drivetrain m_drivetrain;
  static TrapezoidProfile m_profile; 
  TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(15, 20);
  static Timer trapezoidTimer = new Timer();
  double totalDistance;
  /**
   * Creates a new DriveDistance.
   * @param targetDistance distance to drive in feet
   * @param drivetrain is the drivetrain
   */
  public DriveDistance(double targetDistance,Drivetrain drivetrain) {
    

    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> drivetrain.getRightEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> m_profile.calculate(trapezoidTimer.get()).position ,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.ArcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    Shuffleboard.getTab("Distance driving tuning").add(getController());
    Shuffleboard.getTab("Distance driving tuning").addNumber("current distance", ()->drivetrain.getRightEncoder());
    Shuffleboard.getTab("Distance driving tuning").addNumber("target distance", ()->targetDistance);
    m_drivetrain = drivetrain;
   totalDistance = targetDistance;
   getController().setTolerance(1.0/12.0,.05);
  }
  @Override
  public void initialize() {
    trapezoidTimer.reset();
    trapezoidTimer.start();
    m_profile = new TrapezoidProfile(m_Constraints,
      new TrapezoidProfile.State(totalDistance, 0),  
      new TrapezoidProfile.State(0, 0));
    
    m_drivetrain.resetRightEncoder();
    super.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint()&&getController().getSetpoint() == totalDistance;
  }
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("ended driving");
    trapezoidTimer.stop();
    trapezoidTimer.reset();
  }
}

