/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.Map;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Manager for PID operations with a Spark Max motor controller
 */
public class SparkMaxPIDController{
    /**preferred properties for a grid layout given to {@link #addTuningToShuffleboard(ShuffleboardLayout) addTuningToShuffleboard()} */
    public static final Map<String,Object> tuningDisplayMap = Map.of("Number of columns",2,"Number of rows",2,"Label position","LEFT");

    CANSparkMax m_motor;
    CANPIDController m_controller;
    double m_setpoint;
    ControlType m_controlType;

    PIDTuningParameters m_tuning;

    /**
     * Create a new SparkMaxPIDController
     * @param sparkMax the CANSparkMax this controller will manage
     * @param controlType the PID control type, velocity or positional
     * @param tuning the initial PID tuning for the controller
     */
    public SparkMaxPIDController(CANSparkMax sparkMax, ControlType controlType, PIDTuningParameters tuning){
        m_motor = sparkMax;
        m_controller = m_motor.getPIDController();
        m_controlType = controlType;
        m_tuning=tuning;
        
        m_controller.setP(m_tuning.p);
        m_controller.setI(m_tuning.i);
        m_controller.setD(m_tuning.d);
        m_controller.setFF(m_tuning.ff);
    }
    /**
     * Creates a new SparkMaxPIDController with all tuning values set to zero
     * @param sparkMax the CANSparkMax this controller will manage
     * @param controlType the PID control type, velocity or positional
     */
    public SparkMaxPIDController(CANSparkMax sparkMax, ControlType controlType){
        this(sparkMax,controlType,new PIDTuningParameters());
    }
    /**
     * Adds the PID controller tuning parameters to Shuffleboard. As a side effect the Spark Max gains will be updated when 
     * shuffleboard values are changed by a user
     * @param container the layout to display the tuning parameters in
     * @return the layout passed in after tuning parameters have been added
     */
    public ShuffleboardLayout addTuningToShuffleboard(ShuffleboardLayout container){
        //for each value, create a number entry, then add a listener to update the controller when shuffleboard is changed
        container.add("p",m_tuning.p).withPosition(0, 0).getEntry().addListener(notification->setP(notification.value.getDouble()), EntryListenerFlags.kUpdate);
        container.add("i",m_tuning.i).withPosition(1,0).getEntry().addListener(notification->setI(notification.value.getDouble()), EntryListenerFlags.kUpdate);
        container.add("d",m_tuning.d).withPosition(0, 1).getEntry().addListener(notification->setD(notification.value.getDouble()), EntryListenerFlags.kUpdate);
        container.add("ff",m_tuning.ff).withPosition(1, 1).getEntry().addListener(notification->setFF(notification.value.getDouble()), EntryListenerFlags.kUpdate);
        //return the container to make adding properties more convenient 
        return container;
    }
    public void addTuningToNetworkTable(NetworkTable container){
        NetworkTableEntry pEntry = container.getEntry("p");
        pEntry.setDouble(m_tuning.p);
        pEntry.addListener(notification->setP(notification.value.getDouble()), EntryListenerFlags.kUpdate);

        NetworkTableEntry iEntry = container.getEntry("i");
        iEntry.setDouble(m_tuning.i);
        iEntry.addListener(notification->setI(notification.value.getDouble()), EntryListenerFlags.kUpdate);

        NetworkTableEntry dEntry = container.getEntry("d");
        dEntry.setDouble(m_tuning.d);
        dEntry.addListener(notification->setD(notification.value.getDouble()), EntryListenerFlags.kUpdate);

        NetworkTableEntry ffEntry = container.getEntry("ff");
        ffEntry.setDouble(m_tuning.ff);
        ffEntry.addListener(notification->setFF(notification.value.getDouble()), EntryListenerFlags.kUpdate);
    }
    /** Resets the PID controller, clearing the Integral accumulator */
    public void reset(){
        m_controller.setIAccum(0);
        setSetpoint(getSetpoint());
    }


    //gain setters & getters
    /**
     * sets the controller's proportional gain
     * @param newP the new proportional gain
     */
    public void setP(double newP){
        if (m_tuning.p!=newP){
            m_controller.setP(newP);
            m_tuning.p=newP;
        }
    }
    /**
     * sets the controller's integral gain
     * @param newI the new integral gain
     */
    public void setI(double newI){
        if (m_tuning.i!=newI){
            m_controller.setI(newI);
            m_tuning.i=newI;
        }
        
    }
    /**
     * sets the controller's derivative gain
     * @param newD the new derivative gain
     */
    public void setD(double newD){
        if(m_tuning.d!=newD){
            m_controller.setD(newD);
            m_tuning.d=newD;
        }
    }
    /**
     * sets the controller's Feed Forward gain
     * @param newFF the new feed forward gain
     */
    public void setFF(double newFF){
        if(m_tuning.ff!=newFF){
            m_controller.setFF(newFF);
            m_tuning.ff=newFF;
        }
    }
    /**
     * @return the controller's current proportional gain
     */
    public double getP(){
        return m_tuning.p;
    }
     /**
     * @return the controller's current integral gain
     */
    public double getI(){
        return m_tuning.i;
    }
     /**
     * @return the controller's current derivative gain
     */
    public double getD(){
        return m_tuning.d;
    }
     /**
     * @return the controller's current feed forward gain
     */
    public double getFF(){
        return m_tuning.ff;
    }
     /**
     * @return the controller's current setpoint gain
     */
    public double getSetpoint(){
        return m_setpoint;
    }
    /**
     * Sets the controller's setpoint
     * @param newSetpoint the new setpoint in the set control type and the encoder's units
     */
    public void setSetpoint(double newSetpoint){
        m_setpoint=newSetpoint;
        m_controller.setReference(m_setpoint, m_controlType);
    }
}