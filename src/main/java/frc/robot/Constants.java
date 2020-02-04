/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public static final int frontLeftChannel = 0;
    //3
    public static final int rearLeftChannel = 0;
    //2
    public static final int frontRightChannel = 0;
    //1
    public static final int rearRightChannel = 0;
    //4
    public static final int intakeChannel = 11;
    public static final int intakeExtenderChannel = 0;
    public static final int bottomStagingBeltChannel = 12;
    public static final int turretRotationChannel = 5;
    public static final int LauncherLeaderChannel = 1;
    //6
    public static final int LauncherFollowerChannel = 3;
    //7
    //DIO
    public static final int extendedLimitSwitchChannel = 3;
    public static final int retractedLimitSwitchChannel = 4;


    //physical
    public static final double maxTurretAngle = 250;



}
