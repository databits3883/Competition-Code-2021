/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.TurretCameraAim;


/**
 * Add your docs here.
 */
public class Variables {
    private static Variables instance;
    
 
    public static Variables getInstance(){
        if(instance == null){
            System.out.println("running once");
            instance = new Variables();
            instance.navx.zeroYaw();

            initShuffleboard();
            
        }
        return instance;

    }
    static void initShuffleboard(){
        Shuffleboard.getTab("Variables").addNumber("Power cells", instance::getContainedPowerCells);
        Shuffleboard.getTab("Variables").add(instance.m_powerDistributionPanel);
        Shuffleboard.getTab("Variables").addBoolean("shooter at speed", instance::getShooterAtSpeed);
        Shuffleboard.getTab("Variables").addNumber("pitch", instance::getGyroPitch);
        Shuffleboard.getTab("Variables").addNumber("left acceleration", instance::GetXAccel);
        Shuffleboard.getTab("Variables").addNumber("forward acceleration", instance::GetYAccel);
        Shuffleboard.getTab("Variables").addNumber("upwards acceleration", instance::GetZAccel);

        Shuffleboard.getTab("Variables").addBoolean("isAntiTipping", ()->{return instance.getGyroPitch() >= 5;});
    }
    
    AHRS navx = new AHRS(I2C.Port.kMXP );
    public double getGyroAngle(){
        return (navx.getAngle());
        
    }
    public double getGyroPitch(){
        return navx.getPitch();
    }
    public double GetXAccel(){
        return navx.getVelocityX();
    }
    public double GetYAccel(){
        return navx.getVelocityY();
    }
    public double GetZAccel(){
        return navx.getVelocityZ();
    }

    public final CANSparkMax rightLeader = new CANSparkMax(Constants.rightLeaderChannel, MotorType.kBrushless);
    public final CANSparkMax rightFollower = new CANSparkMax(Constants.rightFollowerChannel, MotorType.kBrushless);
    public final CANSparkMax leftLeader = new CANSparkMax(Constants.leftLeaderChannel, MotorType.kBrushless);
    public final CANSparkMax leftFollower= new CANSparkMax(Constants.leftFollowerChannel, MotorType.kBrushless);
    

    private final CANEncoder leftEncoder = new CANEncoder(leftLeader);
    private final CANEncoder rightEncoder = new CANEncoder(rightLeader);
    public double positionalConversion = (7.0/12.0*Math.PI)*(1.0/8.45);
    public double positionalConversionOdom = 1.0/3.28;  // One meter is 3.28 Feet This will convert feet to meters when multiplied. 

    public double GetLeftDistanceMeters(){
       return leftEncoder.getPosition()*positionalConversion;
    }

    public double GetRightDistanceMeters(){
        return rightEncoder.getPosition()*positionalConversion;
     }

    boolean m_isShooterEnabled;
    public boolean getShooterEnabled(){
        return m_isShooterEnabled;
    }
    public void setShooterEnabled(boolean set){
        m_isShooterEnabled = set;
    }

    boolean m_shooterAtSpeed;
    public boolean getShooterAtSpeed(){
        return m_shooterAtSpeed;
    }
    public void setShooterAtSpeed(boolean atSpeed){
        m_shooterAtSpeed = atSpeed;
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
