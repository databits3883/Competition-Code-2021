/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.function.DoubleConsumer;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Controller for PID operations with the Spark Max motor controller
 */
public class SparkMaxPIDController implements Sendable{
    static int instances =0;
    CANSparkMax m_motor;
    CANPIDController m_controller;
    double m_setpoint;
    ControlType m_controlType;
    
    public SparkMaxPIDController(CANSparkMax sparkMax){
        m_motor = sparkMax;
        m_controller = m_motor.getPIDController();
        SendableRegistry.addLW(this,"Spark Max PID Controller",instances++);
    }
    //gain setters & getters
    public void setP(double P){
        m_controller.setP(P);
    }
    public void setI(double I){
        m_controller.setI(I);
    }
    public void setD(double D){
        m_controller.setD(D);
    }
    public void setFF(double FF){
        m_controller.setFF(FF);
    }
    public double getP(){
        return m_controller.getP();
    }
    public double getI(){
        return m_controller.getI();
    }
    public double getD(){
        return m_controller.getD();
    }
    public double getFF(){
        return m_controller.getFF();
    }

    public double getSetpoint(){
        return m_setpoint;
    }
    public void setSetpoint(double newSetpoint){
        m_setpoint=newSetpoint;
        m_controller.setReference(m_setpoint, m_controlType);
    }


    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getFF, this::setFF);

        builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
    }
}
