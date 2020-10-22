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
    public double p;
    public double i;
    public double d;
    public double ff;

    public PIDTuningParameters(double initialP, double initialI, double initialD,double initialFF){
        p = initialP;
        i=initialI;
        d=initialD;
        ff=initialFF;
    }
    public PIDTuningParameters(double initialP, double initialI, double initialD){
        this(initialP,initialI,initialD,0);
    }
    public PIDTuningParameters(){
        this(0,0,0,0);
    }
}
