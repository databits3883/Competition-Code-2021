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
  private final Victor Upperstage = new Victor(2);
  private final Victor Jostler = new Victor(2);

  private final DigitalInput BottomSensor = new DigitalInput(Constants.BottomSensor);
  private final DigitalInput MiddleSensor = new DigitalInput(Constants.MiddleSensor);
  private final DigitalInput TopSensor = new DigitalInput(Constants.TopSensor);

  

  /**
   * Creates a new Staging.
   */
  public Staging() {

  }
  public void Jostle(){
    Jostler.setSpeed(Constants.lowerStagingSpeed);
  }
  public void UpperStage(){
    Upperstage.setSpeed(Constants.upperStagingSpeed);
  }
  public void RunStaging(){
    UpperStage();
    Jostle();
  }
  public void StopStaging(){
    StopUpperStage();
    StopJostle();

  }
  public void StopJostle(){
    Jostler.setSpeed(0);
  }
  public void StopUpperStage(){
    Upperstage.setSpeed(0);
  }
  public void ReverseJostle(){
    Jostler.setSpeed(Constants.upperOuttakeSpeed);
  }
  public void ReverseUpperStage(){
    Upperstage.setSpeed(Constants.upperOuttakeSpeed);
  }
  public void ReverseStaging(){
    ReverseJostle();
    ReverseUpperStage();
  }
  public boolean GetTopSensor(){
    return TopSensor.get();
  }
  public boolean GetMiddleSensor(){
    return TopSensor.get();
  }
  public boolean GetBottomSensor(){
    return TopSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
