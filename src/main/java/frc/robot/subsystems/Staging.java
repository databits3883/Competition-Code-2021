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

/**Stagins subsystem. Manages upper stage and jostler motors, and staging sensors.*/
public class Staging extends SubsystemBase {
  private final Victor Upperstage = new Victor(1);
  private final Victor Jostler = new Victor(2);

  private final DigitalInput BottomSensor = new DigitalInput(Constants.BottomSensor);
  private final DigitalInput MiddleSensor = new DigitalInput(Constants.MiddleSensor);
  private final DigitalInput TopSensor = new DigitalInput(Constants.TopSensor);

  

  /**
   * Creates a new Staging instance.
   */
  public Staging() {

  }
  //TODO: check this doc for accuracy
  /**
   * Moves around power cells in lower stage.
   */
  public void Jostle(){
    Jostler.setSpeed(Constants.lowerStagingSpeed);
  }
  /**Runs the upper stage motor to drive the belt */
  public void UpperStage(){
    Upperstage.setSpeed(Constants.upperStagingSpeed);
  }
  /**Runs both stagin motors, the jostler and upper stage. */
  public void RunStaging(){
    UpperStage();
    Jostle();
  }
  /**Stops both staging motors, the jostler and the upper stage.*/
  public void StopStaging(){
    StopUpperStage();
    StopJostle();

  }
  /**stops the jostler motor. */
  public void StopJostle(){
    Jostler.setSpeed(0);
  }
  /**stops the upper stage motor. */
  public void StopUpperStage(){
    Upperstage.setSpeed(0);
  }
  /**runs jostler motor in reverse for jams. */
  public void ReverseJostle(){
    Jostler.setSpeed(Constants.upperOuttakeSpeed);
  }
  /**runs upper stage backwards for jams. */
  public void ReverseUpperStage(){
    Upperstage.setSpeed(Constants.upperOuttakeSpeed);
  }
  /**runs backwards both staging motors, the jostler and the upper stage.*/
  public void ReverseStaging(){
    ReverseJostle();
    ReverseUpperStage();
  }
  /**gets the current state of the top sensor position
   * @return true if a power cell is in the top sensor position
   */
  public boolean GetTopSensor(){
    return !TopSensor.get();
  }
  /**gets the current state of the middle sensor position
   * @return true if a power cell is in the middle sensor position
   */
  public boolean GetMiddleSensor(){
    return !MiddleSensor.get();
  }
  /**gets the current state of the bottom sensor position
   * @return true if a power cell is in the bottom sensor position
   */
  public boolean GetBottomSensor(){
    return !BottomSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
