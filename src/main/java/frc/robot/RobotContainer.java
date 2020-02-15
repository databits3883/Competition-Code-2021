/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.AdvanceStaging;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.ManualLaunch;
import frc.robot.commands.PositionControl;
import frc.robot.commands.RetractIntake;
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
import frc.robot.subsystems.UpperStagingBelt;
import frc.robot.util.SupplierButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
  private final UpperStagingBelt m_upperStagingBelt = new UpperStagingBelt();
  private final TurretRotator m_turretRotator = new TurretRotator();
  private final Launcher m_launcher = new Launcher();
  private final ControlPanelSpinner m_controlPanelSpinner = new ControlPanelSpinner();
  //private final Rotation m_rotation = new Rotation(m_controlPanelSpinner);
  private final Hood m_hood = new Hood();

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick gunnerJoystick = new Joystick(1);
  private final JoystickButton driverTrigger = new JoystickButton(driverJoystick, 1);
  private final JoystickButton driverButton8 = new JoystickButton(driverJoystick, 8);
  private final JoystickButton gbutton2 = new JoystickButton(gunnerJoystick, 2);
  private final JoystickButton gbutton3 = new JoystickButton(gunnerJoystick, 3);
  private final JoystickButton dbutton6 = new JoystickButton(driverJoystick, 6);
  private final JoystickButton dbutton7 = new JoystickButton(driverJoystick, 7);
  private final XboxController gunnerController = new XboxController(3);
  private final SupplierButton xButton = new SupplierButton( ()->gunnerController.getXButton());
  private final SupplierButton yButton = new SupplierButton( ()->gunnerController.getYButton());
  private final SupplierButton bButton = new SupplierButton( ()->gunnerController.getBButton());  
  private final SupplierButton aButton = new SupplierButton( ()->gunnerController.getAButton()); 
  private final SupplierButton backButton = new SupplierButton( ()->gunnerController.getBackButton());
  private final SupplierButton startButton = new SupplierButton( ()->gunnerController.getStartButton());
  private final SupplierButton leftBumperButton = new SupplierButton( ()->gunnerController.getBumper(Hand.kLeft));
  private final SupplierButton rightBumperButton = new SupplierButton( ()->gunnerController.getBumper(Hand.kRight));
  private final SupplierButton rightTriggButton = new SupplierButton(()-> gunnerController.getTriggerAxis(Hand.kRight)>=0.75);

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
  m_drivetrain.ArcadeDrive(Math.pow(x, 3), Math.pow(y, 3));
  },m_drivetrain );
  private final Command m_runIntake = new InstantCommand(m_intake::intake, m_intake);
  private final Command m_stopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final Command m_autoStopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final Command m_runOutake = new InstantCommand(m_intake::Outake, m_intake).alongWith(new InstantCommand(m_bottomStagingBelt::outTake, m_bottomStagingBelt));
  private final Command m_stopBelt = new InstantCommand(m_bottomStagingBelt::stopBelt, m_bottomStagingBelt);
  private final Command m_advanceStaging = new AdvanceStaging(m_bottomStagingBelt)
      .andThen(new RunCommand(m_bottomStagingBelt::runBelt, m_bottomStagingBelt).withTimeout(0.5))
      
      .andThen(new InstantCommand(m_bottomStagingBelt::stopBelt, m_bottomStagingBelt));
  private final StagingToTop m_stagingToTop = new StagingToTop(m_bottomStagingBelt);
  //private final Command manualTurretPanning = new RunCommand(()-> m_turretRotator.changeAngle(gunnerController.getX(Hand.kLeft)), m_turretRotator);
  ////private final Command manualLauncherWheelspin = new RunCommand(()-> m_launcher.changeSpeed(gunnerController.getTriggerAxis(Hand.kRight)), m_launcher);
  //private final Command manualHoodMovement = new RunCommand(()-> m_hood.changeAngle(gunnerController.getY(Hand.kRight)), m_hood);
  private final Command debugCommand = new InstantCommand(()-> System.out.println("test successful"));
  private final ExtendIntake m_extendIntake = new ExtendIntake(m_intake);
  private final RetractIntake m_retractedIntake = new RetractIntake(m_intake);
  private final ManualLaunch m_manualLaunch = new ManualLaunch(m_upperStagingBelt);
  


  private final SequentialCommandGroup m_initCommand = new SequentialCommandGroup(
    new InstantCommand(m_turretRotator::setCurrentPosition),
    new PrintCommand("init teleop")
  );
  
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
    driverTrigger.whenPressed(m_runIntake);
    driverTrigger.whenReleased(m_stopIntake);
    driverButton8.whenPressed(m_runOutake);
    driverButton8.whenReleased(m_autoStopIntake.alongWith(m_stopBelt));
    //gbutton2.whenPressed(m_stagingToTop,false);
    lowerIntakeTrigger.whenActive(m_advanceStaging);
    //gbutton3.whenPressed(m_rotation);
    xButton.whenActive(m_stagingToTop);
    dbutton6.whenPressed(m_extendIntake);
    dbutton7.whenPressed(m_retractedIntake);
    rightTriggButton.whenPressed(m_manualLaunch);
    
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

  public Command getInitCommand(){
    return m_initCommand;
  }
}
