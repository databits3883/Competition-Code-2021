// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TurretCameraAim;
import frc.robot.subsystems.TurretRotator;

/** Add your docs here. */
public class AutonomousBallChase extends BallFollowing {

    public AutonomousBallChase(Drivetrain drivetrain, TurretRotator turretrotator, TurretCameraAim limelightservo,
            Intake intake) {
        super(drivetrain, turretrotator, limelightservo, intake);
        // TODO Auto-generated constructor stub
    }

    @Override
    double calculatespeed() {
        // TODO Auto-generated method stub
        return 0.1;
    }

    @Override
    void noTargetDrive() {
        // TODO Auto-generated method stub
        m_drivetrain.ArcadeDrive(-0.1, 0);
    }
    
}
