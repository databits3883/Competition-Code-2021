/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BottomStagingBelt;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.TurretRotator;
import frc.robot.subsystems.UpperStagingBelt;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SelectionBasedAutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new SelectionBasedAutoShoot.
   */
  public SelectionBasedAutoShoot(StartPosition startPosition, Hood hood, TurretRotator turretRotator, Launcher launcher, UpperStagingBelt upperStagingBelt, BottomStagingBelt bottomStagingBelt) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
    new FullTurretAim(startPosition.getHoodAngle(), startPosition.getTurretAngle(), startPosition.getLaunchSpeed(), hood, turretRotator, launcher),
    new ShootThreePowerCells(upperStagingBelt, bottomStagingBelt)
    );
  }

  public enum StartPosition{
    kLeft(38.67,77.14,-75), kCenter(39.33,97.14,-75), kRight(39.32,120 ,-75);
    double hoodAngle, turretAngle, launchSpeed;
    StartPosition(double hoodAngle, double turretAngle, double launchSpeed){
      this.hoodAngle = hoodAngle;
      this.turretAngle = turretAngle;
      this.launchSpeed = launchSpeed;
    }
    public double getHoodAngle(){
      return hoodAngle;
    }
    public double getTurretAngle(){
      return turretAngle;
    }
    public double getLaunchSpeed(){
      return launchSpeed;
    }
  }
}
