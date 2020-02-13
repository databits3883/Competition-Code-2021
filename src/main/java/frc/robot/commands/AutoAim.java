/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Variables;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.TurretRotator;
import frc.robot.util.Vector3D;

public class AutoAim extends CommandBase {
  Hood m_hood;
  Launcher m_launcher;
  TurretRotator m_turretRotator;
  /**
   * Creates a new AutoAim.
   */
  public AutoAim(Hood hood, Launcher launcher, TurretRotator turretRotator) {
    m_hood = hood;
    m_launcher = launcher;
    m_turretRotator = turretRotator;
    addRequirements(hood);
    addRequirements(launcher);
    addRequirements(turretRotator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Vector3D targetPosition = getTargetPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
  private Vector3D getTargetPosition(){
   double ta = NetworkTableInstance.getDefault().getTable("limeLight").getEntry("ta").getDouble(0);
   double tx = NetworkTableInstance.getDefault().getTable("limeLight").getEntry("tx").getDouble(0);
   double gyro = Variables.getInstance().getGyroAngle();
   double turret = m_turretRotator.getAngle();
   double distance = 16.4 - 7.42 * Math.log(ta);
   double angle = gyro + turret + tx;
   double x = distance * Math.sin(Math.toRadians(angle));
   double y = distance * Math.cos(Math.toRadians(angle));
   return new Vector3D(x, y, Constants.robotToTargetZDelta);
  }
  
}
