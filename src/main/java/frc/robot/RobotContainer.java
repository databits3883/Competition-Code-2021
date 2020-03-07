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
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.util.SupplierButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final LimelightServo m_limelightServo = new LimelightServo();
  private final Hood m_hood = new Hood();
  private final LEDLights m_ledLights = new LEDLights();
 

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick gunnerJoystick = new Joystick(1);

  private final JoystickButton driverTrigger = new JoystickButton(driverJoystick, 1);
  private final JoystickButton gbutton2 = new JoystickButton(gunnerJoystick, 2);
  private final JoystickButton gbutton3 = new JoystickButton(gunnerJoystick, 3);
  private final JoystickButton driverButton4 = new JoystickButton(driverJoystick, 4);
  private final JoystickButton driverButton5 = new JoystickButton(driverJoystick, 5);
  private final JoystickButton dbutton6 = new JoystickButton(driverJoystick, 6);
  private final JoystickButton dbutton7 = new JoystickButton(driverJoystick, 7);

  private final JoystickButton driverButton8 = new JoystickButton(driverJoystick, 8);
  private final JoystickButton driverButton9 = new JoystickButton(driverJoystick, 9);
  private final JoystickButton driverButton10 = new JoystickButton(driverJoystick, 10);
  private final JoystickButton driverButton11 = new JoystickButton(driverJoystick, 11);


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
  private final SupplierButton driverEitherSideButton = new SupplierButton(()-> driverButton4.get() || driverButton5.get());

  private final Trigger lowerIntakeTrigger = new Trigger(){
    @Override
    public boolean get(){
      return m_bottomStagingBelt.getSensorBottom();
    }
  };


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  private final SlewRateLimiter driverYLimiter = new SlewRateLimiter(0.7);
  
  private final double joystickDeadband=(Math.pow(.07,3));
  private final Command manualArcadeDrive = new RunCommand(()->{
    double x = Math.pow(driverJoystick.getX(),3);
    double y = driverYLimiter.calculate(-Math.pow(driverJoystick.getY(),3));
    if( Math.abs(x)<joystickDeadband){
      x=0;
      }
      else {
        x-=Math.copySign(joystickDeadband, x);
      }
    if( Math.abs(y)<joystickDeadband){
       y=0;
      }
      else{
       y-=Math.copySign(joystickDeadband, y);
      }
  m_drivetrain.ArcadeDrive(x, y);
  },m_drivetrain );
  private final Command m_runIntake = new InstantCommand(m_intake::intake, m_intake).alongWith();
  private final Command m_turnto90 =new InstantCommand(()->m_turretRotator.setAngle(90),m_turretRotator);
  private final Command m_stopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final Command m_autoStopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final Command m_runOutake = new InstantCommand(m_intake::Outake, m_intake).alongWith(new InstantCommand(m_bottomStagingBelt::outTake, m_bottomStagingBelt)).alongWith(new InstantCommand(m_upperStagingBelt::outTake,m_upperStagingBelt));
  private final Command m_stopBelt = new InstantCommand(m_bottomStagingBelt::stopBelt, m_bottomStagingBelt).alongWith(new InstantCommand(m_upperStagingBelt::stopBelt, m_upperStagingBelt));
  private final Command m_advanceStaging = new AdvanceStaging(m_bottomStagingBelt)
      .andThen(new RunCommand(m_bottomStagingBelt::runBelt, m_bottomStagingBelt).withTimeout(0.15))
      
      .andThen(new InstantCommand(m_bottomStagingBelt::stopBelt, m_bottomStagingBelt));
  private final StagingToTop m_stagingToTop = new StagingToTop(m_bottomStagingBelt);

  private final BallFollowing m_ballfollowing = new BallFollowing(m_drivetrain, m_turretRotator, m_limelightServo, m_intake);

  private final double turretJoystickDeadband = 0.08;
  private final Command m_manualTurretPanning = new RunCommand(()->{
    double stickValue = gunnerController.getX(Hand.kLeft);
    //System.out.println(stickValue);
    if( Math.abs(stickValue)<turretJoystickDeadband){
      stickValue=0;
      }
      else {
        stickValue-=Math.copySign(turretJoystickDeadband, stickValue);
      }
     m_turretRotator.changeAngle(stickValue*(Constants.maxTurretVelocity/50.0));
  }, m_turretRotator);
  private final Command m_manualLauncherWheelSpin = new RunCommand(()-> m_launcher.setSpeed(gunnerController.getTriggerAxis(Hand.kLeft)*-150), m_launcher);
  private final Command m_manualHoodMovement = new RunCommand(()-> {
    double stickValue = gunnerController.getY(Hand.kRight);
    if( Math.abs(stickValue)<turretJoystickDeadband){
      stickValue=0;
      }
      else {
        stickValue-=Math.copySign(turretJoystickDeadband, stickValue);
      }
    m_hood.changeAngle(stickValue*(1.0/12.0));
  }, m_hood);

  private final Command debugCommand = new InstantCommand(()-> System.out.println("test successful"));
  private final Command m_extendIntake = new ExtendIntake(m_intake).withTimeout(5);;
  private final Command m_retractedIntake = new RetractIntake(m_intake).withTimeout(5);;
  private final Command m_manualLaunch = new ManualLaunch(m_upperStagingBelt, m_bottomStagingBelt).withInterrupt(()-> !Variables.getInstance().getShooterEnabled());
  private final Command m_turnServo = new RunCommand(()->m_limelightServo.deltaPosition(gunnerController.getY(Hand.kLeft)/50.0*180.0), m_limelightServo);

  private final AcquireTarget m_acquireTarget = new AcquireTarget(m_limelightServo, m_turretRotator, m_hood,m_launcher);
  private final ShootThreePowerCells m_shootThreePowerCells = new ShootThreePowerCells(m_upperStagingBelt, m_bottomStagingBelt);
  private final RevLauncher m_revLauncher70 = new RevLauncher(70, m_launcher);
  private final RevLauncher m_revLauncher0 = new RevLauncher(0, m_launcher);
  
  private final SetHoodAngle m_setHoodAngle5 = new SetHoodAngle(5, m_hood);
  private final SetHoodAngle m_setHoodAngle25 = new SetHoodAngle(25, m_hood);

  private final SequentialCommandGroup m_initCommand = new SequentialCommandGroup(
    new InstantCommand(m_turretRotator::setCurrentPosition, m_turretRotator),
    new InstantCommand(m_hood::setCurrentPosition,m_hood),
    new PrintCommand("init teleop")
  );

  private final Command m_emergencyStopDrivetrain = new RunCommand(()->m_drivetrain.EmergencyStop(), m_drivetrain);
  private final ManualIntakeMove m_manualRaiseIntake = new ManualIntakeMove(1, m_intake);
  private final ManualIntakeMove m_manualLowerIntake = new ManualIntakeMove(-1, m_intake);
 
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
    m_limelightServo.setDefaultCommand(m_turnServo);
    m_hood.setDefaultCommand(m_manualHoodMovement);
    m_turretRotator.setDefaultCommand(m_manualTurretPanning);
    //m_launcher.setDefaultCommand(m_manualLauncherWheelSpin);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverEitherSideButton.whileHeld(m_emergencyStopDrivetrain, false);
    driverTrigger.whenPressed(m_runIntake);
    driverTrigger.whenReleased(m_stopIntake);
    driverButton8.whileHeld(m_runOutake, false);
    driverButton8.whenReleased(m_autoStopIntake.alongWith(m_stopBelt));
    //driverButton9.whenPressed(m_shootThreePowerCells);
    driverButton10.whileHeld(m_manualLowerIntake);
    driverButton11.whileHeld(m_manualRaiseIntake);

    //gbutton2.whenPressed(m_stagingToTop,false);
    lowerIntakeTrigger.whenActive(m_advanceStaging);
    //gbutton3.whenPressed(m_rotation);
    xButton.toggleWhenPressed(m_stagingToTop);
    dbutton6.whenPressed(m_extendIntake);
    dbutton7.whenPressed(m_retractedIntake);
    rightTriggButton.whileHeld(m_manualLaunch);

    startButton.toggleWhenActive(m_acquireTarget);
    driverButton9.whileHeld(m_turnto90);
    

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return new ShootAndMoveAutoBasic(m_hood, m_turretRotator, m_launcher, m_upperStagingBelt, m_bottomStagingBelt, m_drivetrain);
  }

  public Command getInitCommand(){
    return m_initCommand;
  }

  public Command getDisabledCommand(){
    return new ParallelCommandGroup(
      new RunCommand(m_turretRotator::setCurrentPosition, m_turretRotator),
      new RunCommand(()->m_launcher.setSpeed(0), m_launcher),
      new RunCommand(m_hood::setCurrentPosition, m_hood)
    ){@Override
      public boolean runsWhenDisabled(){
        return true;
      }
    };
  }
}
