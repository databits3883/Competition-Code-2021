/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  /**
  * Creates a new Hood.
  */  
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.hookChannel, MotorType.kBrushless);
  private final CANPIDController controller = new CANPIDController(hoodMotor);
  private final CANEncoder encoder = new CANEncoder(hoodMotor);
  private double p,i,d,ff,position;
  private NetworkTableEntry pEntry,iEntry,dEntry,ffEntry,positionEntry;
  public Hood() {
    encoder.setVelocityConversionFactor(1);

    pEntry = Shuffleboard.getTab("hoodTuning").add("portional",p).getEntry();
    iEntry = Shuffleboard.getTab("hoodTuning").add("integral",i).getEntry();
    dEntry = Shuffleboard.getTab("hoodTuning").add("derivative",d).getEntry();
    ffEntry = Shuffleboard.getTab("hoodTuning").add("feedForward",ff).getEntry();



    positionEntry = Shuffleboard.getTab("hoodTuning").add("position",position).getEntry();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
