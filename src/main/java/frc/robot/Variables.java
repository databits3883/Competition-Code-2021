/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


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

            initShuffleboard();
            
        }
        return instance;

    }
    static void initShuffleboard(){
        Shuffleboard.getTab("Variables").addNumber("Power cells", instance::getContainedPowerCells);
        Shuffleboard.getTab("Variables").add(instance.m_powerDistributionPanel);
        Shuffleboard.getTab("Variables").addBoolean("shooter at speed", instance::getShooterAtSpeed);
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

    boolean m_shooterAtSpeed;
    public boolean getShooterAtSpeed(){
        return m_shooterAtSpeed;
    }
    public void setShooterAtSpeed(boolean atSpeed){
        m_shooterAtSpeed = atSpeed;
    }

    int containedPowerCells =0;
    public void addPowerCell(){
        containedPowerCells++;
    }
    public void subtractPowerCell(){
        containedPowerCells--;
    }
    public int getContainedPowerCells(){
        return containedPowerCells;
    }

    PowerDistributionPanel m_powerDistributionPanel= new PowerDistributionPanel();;
    public double getPDPCurrent(int channel){
        return m_powerDistributionPanel.getCurrent(channel);
    }
}
