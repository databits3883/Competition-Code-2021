/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class AccelerationLimiter {
    double maxAcceleration;
    double targetSetpoint;
    double currentSetpoint;
    Timer timer;
    public AccelerationLimiter(double maxAcceleration){
        this.maxAcceleration = maxAcceleration;
        currentSetpoint = 0;
    }
    public void setTarget(double newTarget){
        targetSetpoint = newTarget;
        timer = new Timer();
        timer.start();
    }
    public double get(){
        double timeDelta = timer.get();
        timer.reset();
        double targetChange = targetSetpoint - currentSetpoint;
        if(Math.abs(targetChange)/timeDelta > maxAcceleration){
            currentSetpoint+= Math.copySign(maxAcceleration*timeDelta, targetChange);
            System.out.println(currentSetpoint+" "+timeDelta);
        }
        else currentSetpoint += targetChange; 
        return currentSetpoint;
    }
}
