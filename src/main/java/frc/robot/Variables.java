/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;


/**
 * Add your docs here.
 */
public class Variables {
    private static Variables instance;
 
    public static Variables getInstance(){
        if(instance == null){
            System.out.println("running once");
            instance = new Variables();
            instance.navx.zeroYaw();
        }
        return instance;

    }
    
    AHRS navx = new AHRS(I2C.Port.kMXP );
    
   
    public double getGyroAngle(){
        return (navx.getAngle());
    }

    boolean m_isShooterEnabled;
    public boolean getShooterEnabled(){
        return m_isShooterEnabled;
    }
    public void setShooterEnabled(boolean set){
        m_isShooterEnabled = set;
    }
}
