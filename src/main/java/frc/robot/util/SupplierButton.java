/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Add your docs here.
 */
public class SupplierButton extends Button{
    private final BooleanSupplier m_supplier;
    public SupplierButton(BooleanSupplier supplier){
        m_supplier = supplier;
    }

    @Override
    public boolean get(){
        return m_supplier.getAsBoolean();
    }
    
}
