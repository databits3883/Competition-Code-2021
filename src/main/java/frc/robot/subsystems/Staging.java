/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Staging extends SubsystemBase {
  private final Victor TowerMotor = new Victor(1);
  private final Victor JostlerMotor = new Victor(2);

  private final DigitalInput BottomSensor = new DigitalInput(Constants.BottomSensor);
  private final DigitalInput MiddleSensor = new DigitalInput(Constants.MiddleSensor);
  private final DigitalInput TopSensor = new DigitalInput(Constants.TopSensor);

  

  /**
   * Creates a new Staging.
   */
  public Staging() {

  }
  /**
   * Enable the mechanism to prevent PowerCell jamming in the intake.
   */
  public void Jostle(){
    JostlerMotor.setSpeed(Constants.lowerStagingSpeed);
  }
  public void MovePowerCelltoLauncher(){
    TowerMotor.setSpeed(Constants.upperStagingSpeed);
  }
  public void RunStaging(){
    MovePowerCelltoLauncher();
    Jostle();
  }
  public void StopStaging(){
    MovePowerCellStop();
    StopJostle();

  }
  public void StopJostle(){
    JostlerMotor.setSpeed(0);
  }
  public void MovePowerCellStop(){
    TowerMotor.setSpeed(0);
  }
  public void ReverseJostle(){
    JostlerMotor.setSpeed(Constants.upperOuttakeSpeed);
  }
  public void MovePowerCelltoIntake(){
    TowerMotor.setSpeed(Constants.upperOuttakeSpeed);
  }
  public void ReverseStaging(){
    ReverseJostle();
    MovePowerCelltoIntake();
  }
  public boolean GetTopSensor(){
    return !TopSensor.get();
  }
  public boolean GetMiddleSensor(){
    return !MiddleSensor.get();
  }
  public boolean GetBottomSensor(){
    return !BottomSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
