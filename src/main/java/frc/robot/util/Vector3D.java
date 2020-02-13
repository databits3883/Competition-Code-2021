/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class Vector3D {
    private double x;
    private double y;
    private double z;
    public Vector3D(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }
    public double getX(){
        return x;
    }
    /**
     * @param x the x to set
     */
    public void setX(double x) {
        this.x = x;
    }
    public double getY(){
        return y;
    }
    /**
     * @param y the y to set
     */
    public void setY(double y) {
        this.y = y;
    }
    public double getZ(){
        return z;
    }
    /**
     * @param z the z to set
     */
    public void setZ(double z) {
        this.z = z;
    }
}
