/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN
    public static final int leftLeaderChannel = 1;
    public static final int leftFollowerChannel = 4;
    public static final int rightLeaderChannel = 2;
    public static final int rightFollowerChannel = 3;
    
    public static final int intakeExtenderChannel = 10;
 
    public static final int turretRotationChannel = 5;
    public static final int turretHoodChannel =8;
    public static final int launcherLeaderChannel = 6;
    public static final int launcherFollowerChannel = 7;
  
    public static final int controlWheelChannel = 9;

    //pwm
    public static final int intakeRetrieveChannel = 0;
    public static final int lowerIntakeChannel = 1;
    public static final int upperIntakeChannel = 2;
    public static final int hookLiftChannel = 3;
    public static final int liftWinchChannel = 4;
    public static final int cameraServo = 5;
    public static final int ledChannel = 6;

    //DIO
    public static final int TopSensor = 9+10;
    public static final int BottomSensor = 8+10;
    public static final int MiddleSensor = 7+10;

    public static final int controlPanelPressedChannel = 5;

    //I2C
    public static final I2C.Port colorSensorPort = Port.kOnboard;

    //physical
    //distances in feet, time in seconds
    public static final double maxTurretAngle = 278;
    public static final double maxDriveSpeed = 20;
    public static final double wheelDiameter = 7.0/12.0;
    public static final double driveTrainGearingRatio = 8.45;
    public static final double minimumHoodAngle = 17.6;
    public static final double maximumHoodAngle = 26.2+minimumHoodAngle;
    public static final double lowerStagingSpeed = -0.75;
    public static final double upperStagingSpeed = -1;
    public static final double upperOuttakeSpeed = .4;
    public static final double Raiseintakespeed =.3;
    public static final double loweringIntakeSpeed = -.3;
    public static final double intakeSpeed = 1;

    public static final double maxTurretVelocity = 120; //degrees per second
    public static final double maxHoodVelocity = 3000; //degrees per second

    public static final double minimumShootSpeed = 25;
    
    //Color
    public static final double redThreshold = 0.3;
    public static final double greenThreshold = 0.55;
    public static final double blueThreshold = 0.255;

    
    }
