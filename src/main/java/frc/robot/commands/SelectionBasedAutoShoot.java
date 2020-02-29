/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SelectionBasedAutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new SelectionBasedAutoShoot.
   */
  public SelectionBasedAutoShoot() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
  }

  public enum StartPosition{
    kLeft(0,0,0), kCenter(0,0,0), kRight(0,0,0);
    double hoodAngle, turretAngle, launchSpeed;
    StartPosition(double hoodAngle, double turretAngle, double launchSpeed){
      this.hoodAngle = hoodAngle;
      this.turretAngle = turretAngle;
      this.launchSpeed = launchSpeed;
    }
  }
}
