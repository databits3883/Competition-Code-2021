/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;


import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Controller for PID operations with the Spark Max motor controller
 */
public class SparkMaxPIDController{
    CANSparkMax m_motor;
    CANPIDController m_controller;
    double m_setpoint;
    ControlType m_controlType;

    double p;
    double i;
    double d;
    double ff;
    
    public SparkMaxPIDController(CANSparkMax sparkMax, ControlType controlType){
        m_motor = sparkMax;
        m_controller = m_motor.getPIDController();
        m_controlType = controlType;
    }
    //gain setters & getters
    public void setP(double P){
        if (p!=P){
            m_controller.setP(P);
            p=P;
        }
    }
    public void setI(double I){
        if (i!=I){
            m_controller.setI(I);
            i=I;
        }
        
    }
    public void setD(double D){
        if(d!=D){
            m_controller.setD(D);
            d=D;
        }
    }
    public void setFF(double FF){
        if(ff!=FF){
            m_controller.setFF(FF);
            ff=FF;
        }
    }
    public double getP(){
        return p;
    }
    public double getI(){
        return i;
    }
    public double getD(){
        return d;
    }
    public double getFF(){
        return ff;
    }

    public double getSetpoint(){
        return m_setpoint;
    }
    public void setSetpoint(double newSetpoint){
        m_setpoint=newSetpoint;
        m_controller.setReference(m_setpoint, m_controlType);
    }
}