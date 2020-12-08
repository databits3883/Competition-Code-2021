/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.NetworkTablesUpdater;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Add your docs here.
 */
public class NetworkTablesUpdaterRegistry {
    static NetworkTablesUpdaterRegistry instance;

    ArrayList<NetworkTablesUpdater> m_registered;

    private NetworkTablesUpdaterRegistry(){
        m_registered = new ArrayList<>(1);
    }
    public static NetworkTablesUpdaterRegistry getInstance(){
        if(instance !=null){
            return instance;
        }else{
            instance = new NetworkTablesUpdaterRegistry();
            return instance;
        }
    }

    public void periodic(){
        m_registered.forEach(updater->updater.update());
    }

    public void addUpdate(NetworkTableEntry entry, Supplier<Object> supplier){
        m_registered.add(new NetworkTablesUpdater(entry,supplier));
    }
}
