/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Data Object for hodling PID tuning values
 */
public class PIDTuningParameters {
    /**The current proportional gain of this parameter set*/
    public double p;
    /**The current integral gain of this parameter set*/
    public double i;
    /**The current derivative gain of this parameter set*/
    public double d;
    /**The current feed forward gain of this parameter set*/
    public double ff;
/**
 * Create a new parameters object with preset values, including a feed forward gain
 * @param initialP the initial Proportional gain
 * @param initialI the initial Integral gain
 * @param initialD the initial Derivative gain
 * @param initialFF the initial feed forward gain
 */
    public PIDTuningParameters(double initialP, double initialI, double initialD,double initialFF){
        p = initialP;
        i=initialI;
        d=initialD;
        ff=initialFF;
    }
    /**
 * Create a new parameters object with preset values with no feed forward gain
 * @param initialP the initial Proportional gain
 * @param initialI the initial Integral gain
 * @param initialD the initial Derivative gain
     */
    public PIDTuningParameters(double initialP, double initialI, double initialD){
        this(initialP,initialI,initialD,0);
    }
    /**Create a new parameters oject with all gains set to zero */
    public PIDTuningParameters(){
        this(0,0,0,0);
    }
}
