/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.NetworkTablesUpdater;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Add your docs here.
 */
public class NetworkTablesUpdater {
    NetworkTableEntry m_entry;
    Supplier<Object> m_supplier;
    public void update(){
        m_entry.setValue(m_supplier.get());
    }

    public NetworkTablesUpdater(NetworkTableEntry entry, Supplier<Object> supplier){
        m_entry=entry;
        m_supplier = supplier;
    }
}
