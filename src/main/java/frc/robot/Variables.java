/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Variables {
    private static Variables instance;

    public static Variables getInstance(){
        if(instance == null){
            System.out.println("running once");
            instance = new Variables();

            initShuffleboard();
            
        }
        return instance;

    }
    
 
    static void initShuffleboard(){
        Shuffleboard.getTab("Variables").addNumber("Power cells", instance::getContainedPowerCells);
        Shuffleboard.getTab("Variables").add(instance.m_powerDistributionPanel);
        //Shuffleboard.getTab("Variables").addBoolean("shooter at speed", instance::getShooterAtSpeed);
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

    
    public double DistanceFromPowerPortMeters(double angle){
        
        double d;
        double robotAndReflectorHeightDifference = Constants.powerPortTopReflectorAltitude - Constants.cameraHeight;
        
        d = robotAndReflectorHeightDifference/Math.cos(angle);
        return d;

    }

    PowerDistributionPanel m_powerDistributionPanel= new PowerDistributionPanel();;
    public double getPDPCurrent(int channel){
        return m_powerDistributionPanel.getCurrent(channel);
    }
    
}