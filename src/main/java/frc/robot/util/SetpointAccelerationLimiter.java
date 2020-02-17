/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Add your docs here.
 */
public class SetpointAccelerationLimiter {
    double m_maxAcceleration;
    double m_maxVelocity;
    double targetSetpoint;
    double currentSetpoint;
    double currentVelocity;
    Timer m_timer;
    public SetpointAccelerationLimiter(double maxVelocity, double maxAcceleration){
        m_maxAcceleration = maxAcceleration;
        m_maxVelocity = maxVelocity;
        m_timer = new Timer();
        m_timer.start();
    }
    public void setSetpoint(double newSetpoint){
        targetSetpoint = newSetpoint;
        m_timer.reset();
    }
    public double get(){
        double deltaTime = m_timer.get();
        m_timer.reset();

        double targetVelocity = (targetSetpoint - currentSetpoint)/deltaTime;
        
        targetVelocity = MathUtil.clamp(targetVelocity, -m_maxVelocity, m_maxVelocity);
        //double targetAcceleration = (targetVelocity - currentVelocity)/deltaTime;
        //targetAcceleration = MathUtil.clamp(targetAcceleration, -m_maxAcceleration, m_maxAcceleration);
        //currentVelocity += targetAcceleration *deltaTime;
        //currentSetpoint += currentVelocity*deltaTime;
        currentSetpoint += targetVelocity*deltaTime;
        return currentSetpoint;
    }
}
