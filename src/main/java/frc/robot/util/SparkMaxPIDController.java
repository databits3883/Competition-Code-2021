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

    PIDTuningParameters m_tuning;

    
    public SparkMaxPIDController(CANSparkMax sparkMax, ControlType controlType, PIDTuningParameters tuning){
        m_motor = sparkMax;
        m_controller = m_motor.getPIDController();
        m_controlType = controlType;
        m_tuning=tuning;
    }
    public SparkMaxPIDController(CANSparkMax sparkMax, ControlType controlType){
        this(sparkMax,controlType,new PIDTuningParameters());
    }
    //gain setters & getters
    public void setP(double P){
        if (m_tuning.p!=P){
            m_controller.setP(P);
            m_tuning.p=P;
        }
    }
    public void setI(double I){
        if (m_tuning.i!=I){
            m_controller.setI(I);
            m_tuning.i=I;
        }
        
    }
    public void setD(double D){
        if(m_tuning.d!=D){
            m_controller.setD(D);
            m_tuning.d=D;
        }
    }
    public void setFF(double FF){
        if(m_tuning.ff!=FF){
            m_controller.setFF(FF);
            m_tuning.ff=FF;
        }
    }
    public double getP(){
        return m_tuning.p;
    }
    public double getI(){
        return m_tuning.i;
    }
    public double getD(){
        return m_tuning.d;
    }
    public double getFF(){
        return m_tuning.ff;
    }

    public double getSetpoint(){
        return m_setpoint;
    }
    public void setSetpoint(double newSetpoint){
        m_setpoint=newSetpoint;
        m_controller.setReference(m_setpoint, m_controlType);
    }
}