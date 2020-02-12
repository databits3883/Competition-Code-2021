/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.AdvanceStaging;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Rotation;
import frc.robot.commands.StagingToTop;
import frc.robot.subsystems.BottomStagingBelt;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.TurretRotator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final BottomStagingBelt m_bottomStagingBelt = new BottomStagingBelt();
  private final TurretRotator m_turretRotator = new TurretRotator();
  private final Launcher m_launcher = new Launcher();
  private final ControlPanelSpinner m_controlPanelSpinner = new ControlPanelSpinner();
  private final Rotation m_rotation = new Rotation(m_controlPanelSpinner);
  private final Hood m_hood = new Hood();

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick gunnerJoystick = new Joystick(1);
  private final JoystickButton gbutton1 = new JoystickButton(gunnerJoystick, 1);
  private final JoystickButton gbutton2 = new JoystickButton(gunnerJoystick, 2);
  private final JoystickButton gbutton3 = new JoystickButton(gunnerJoystick, 3);
  private final XboxController gunnerController = new XboxController(0);
  private final Trigger lowerIntakeTrigger = new Trigger(){
    @Override
    public boolean get(){
      return m_bottomStagingBelt.getSensorBottom();
    }
  };


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Command manualArcadeDrive = new RunCommand(()->{
    double x = driverJoystick.getX();
    double y = -driverJoystick.getY();
    if( Math.abs(x)<.07){
      x=0;
      }
      else {
        x-=Math.copySign(.07, x);
      }
    if( Math.abs(y)<.07){
       y=0;
      }
      else{
       y-=Math.copySign(.07, y);
      }
  m_drivetrain.ArcadeDrive(x, y);
  },m_drivetrain );
  private final Command m_runIntake = new InstantCommand(m_intake::intake, m_intake);
  private final Command m_stopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final AdvanceStaging m_advanceStaging = new AdvanceStaging(m_bottomStagingBelt);
  private final StagingToTop m_stagingToTop = new StagingToTop(m_bottomStagingBelt);
  private final Command manualTurretPanning = new RunCommand(()-> m_turretRotator.changeAngle(gunnerJoystick.getX()), m_turretRotator);
  private final Command manualLauncherWheelspin = new RunCommand(()-> m_launcher.changeSpeed(gunnerController.getTriggerAxis(Hand.kRight)), m_launcher);
  private final Command manualHoodMovement = new RunCommand(()-> m_hood.changeAngle(gunnerController.getY(Hand.kRight)), m_hood);
  
  


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set Default Commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void setDefaultCommands(){
    m_drivetrain.setDefaultCommand(manualArcadeDrive);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    gbutton1.whenPressed(m_runIntake);
    gbutton1.whenReleased(m_stopIntake);
    gbutton2.whenPressed(m_stagingToTop,false);
    lowerIntakeTrigger.whileActiveContinuous(m_advanceStaging);
    gbutton3.whenPressed(m_rotation);
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
