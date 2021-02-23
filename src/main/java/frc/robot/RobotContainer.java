/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.Odometry;

import frc.robot.util.SupplierButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Odometry m_Odometry =new Odometry();

  private final Drivetrain       m_drivetrain =       new Drivetrain(m_Odometry);
  private final Intake           m_intake =           new Intake();
  private final Staging          m_staging =          new Staging();
  private final TurretRotator    m_turretRotator =    new TurretRotator();
  private final TurretLauncher   m_launcher =         new TurretLauncher();
  //private final ControlPanelSpinner m_controlPanelSpinner = new ControlPanelSpinner();
  private final TurretCameraAim  m_limelightServo =   new TurretCameraAim();
  private final TurretHood       m_hood =             new TurretHood();
  private final LEDLights        m_ledLights =        new LEDLights();
  private final Climb            m_climb =            new Climb();
  private final RaiseHook        m_raiseHook =        new RaiseHook(m_climb);
  private final LowerHook        m_lowerHook =        new LowerHook(m_climb);
  private final FindLocation     m_findLocation =     new FindLocation(m_drivetrain,m_limelightServo);

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick gunnerJoystick = new Joystick(1);

  private final JoystickButton driverTrigger = new JoystickButton(driverJoystick, 1);
  private final JoystickButton gunnerbutton2 = new JoystickButton(gunnerJoystick, 2);
  private final JoystickButton gunnerbutton3 = new JoystickButton(gunnerJoystick, 3);
  private final JoystickButton driverbutton2 = new JoystickButton(driverJoystick, 2);
  private final JoystickButton driverbutton3 = new JoystickButton(driverJoystick, 3);
  private final JoystickButton driverButton4 = new JoystickButton(driverJoystick, 4);
  private final JoystickButton driverButton5 = new JoystickButton(driverJoystick, 5);
  private final JoystickButton driverbutton6 = new JoystickButton(driverJoystick, 6);
  private final JoystickButton driverbutton7 = new JoystickButton(driverJoystick, 7);

  

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

  
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  private final SlewRateLimiter driverYLimiter = new SlewRateLimiter(0.7);
  
  private final double joystickDeadband=0.07;
  private final double joyStickDeadbandCompliment = 1-joystickDeadband;
  public double driveDampeningX = 8;
  public double driveDampeningY = 8;
  public double CurveStick(double joyVal, double dampening){
    if(dampening!=0){
      return (Math.tan(joyVal* Math.atan(dampening)))/dampening;
    }
    return joyVal;
  }

  

  private final Command manualArcadeDrive = new RunCommand(()->{
    double x = driverJoystick.getX();
    double y = -driverJoystick.getY();
    if( Math.abs(x)<joystickDeadband){
      x=0;
      }
      else {
        x-=Math.copySign(joystickDeadband, x);
        x/=joyStickDeadbandCompliment;
      }
    if( Math.abs(y)<joystickDeadband){
       y=0;
      }
      else{
       y-=Math.copySign(joystickDeadband, y);
       y/=joyStickDeadbandCompliment;
      }
    //double x = Math.pow(driverJoystick.getX(),5);
    x = CurveStick(x, driveDampeningX);
    //double y = driverYLimiter.calculate(-Math.pow(driverJoystick.getY(),3));
    //double y = -Math.pow(driverJoystick.getY(),5);
    y = CurveStick(y, driveDampeningY);
    
  m_drivetrain.ArcadeDrive(x, y);
  },m_drivetrain );

  private final Command m_runIntake = new InstantCommand(m_intake::intake, m_intake).alongWith();

  private final Command m_turnto90 =new InstantCommand(()->m_turretRotator.setAngle(90),m_turretRotator);
  private final Command gyroReset = new InstantCommand(()->Variables.getInstance().resetNavx());
  private final Command odometryReset = new InstantCommand(()->m_drivetrain.ResetOdometry());

  private final Command m_stopIntake = new InstantCommand(m_intake::stop, m_intake);
  private final Command m_autoStopIntake = new InstantCommand(m_intake::stop, m_intake);

  private final Command m_advanceStaging = new AutoAdvanceStaging(m_staging);

  private final Command m_runOutake = new InstantCommand(m_intake::Outake, m_intake).alongWith(new InstantCommand(m_staging::ReverseStaging,m_staging));
  private final Command m_stopBelt = new InstantCommand(m_staging::StopStaging, m_staging);
  
  private final StagingToTop m_stagingToTop = new StagingToTop(m_staging);

  //private final BallFollowing m_ballfollowing = new BallFollowing(m_drivetrain, m_turretRotator, m_limelightServo, m_intake);

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
  private final Command m_manualLaunch = new ManualLaunch(m_staging).withInterrupt(()-> !Variables.getInstance().getShooterEnabled());
  private final Command m_turnServo = new RunCommand(()->{
    double command = gunnerController.getY(Hand.kLeft);
    if(Math.abs(command )<= 0.1) command =0;
    m_limelightServo.deltaPosition(command/50.0*180.0);}, m_limelightServo);

  private final AcquireTarget m_acquireTarget = new AcquireTarget(m_limelightServo, m_turretRotator, m_hood,m_launcher);
  private final ShootThreePowerCells m_shootThreePowerCells = new ShootThreePowerCells(m_staging);
  private final TeleopBallFollowing m_teleopBallFollowing = new TeleopBallFollowing(driverJoystick, m_drivetrain, m_turretRotator, m_limelightServo, m_intake);
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
 
  SendableChooser<Command> chooser = new SendableChooser<Command>();
  
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Configure NetworkTables config
    initConfig();
    // Set Default Commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
    addAutonomous();
  }

  private void setDefaultCommands(){
    m_drivetrain.setDefaultCommand(manualArcadeDrive);
    
    m_limelightServo.setDefaultCommand(m_turnServo);
    m_hood.setDefaultCommand(m_manualHoodMovement);
    m_turretRotator.setDefaultCommand(m_manualTurretPanning);
    //m_launcher.setDefaultCommand(m_manualLauncherWheelSpin);
    m_staging.setDefaultCommand(m_advanceStaging);


  }
  //IN the future these could be set as persistant
  /** Setup for networkTables config values */
  void initConfig(){
    NetworkTableEntry driveDampeningXEntry = NetworkTableInstance.getDefault().getTable("config").getEntry("Drive X Dampening");
    driveDampeningXEntry.setDouble(driveDampeningX);
    driveDampeningXEntry.addListener(e->driveDampeningX=e.value.getDouble(), EntryListenerFlags.kUpdate);

    NetworkTableEntry driveDampeningYEntry = NetworkTableInstance.getDefault().getTable("config").getEntry("Drive Y Dampening");
    driveDampeningYEntry.setDouble(driveDampeningY);
    driveDampeningYEntry.addListener(e->driveDampeningY=e.value.getDouble(), EntryListenerFlags.kUpdate);
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverbutton2.whileHeld(m_lowerHook);
    driverbutton3.whileHeld(m_raiseHook);
    driverButton4.whileHeld(m_teleopBallFollowing);
    driverButton5.whileHeld(m_emergencyStopDrivetrain, false);
    driverTrigger.whenPressed(m_runIntake);
    driverTrigger.whenReleased(m_stopIntake);
    driverButton8.whileHeld(m_runOutake, false);
    driverButton8.whenReleased(m_autoStopIntake.alongWith(m_stopBelt));
    //driverButton9.whenPressed(m_shootThreePowerCells);
    driverButton10.whileHeld(m_manualLowerIntake);
    driverButton11.whileHeld(m_manualRaiseIntake);

    //gbutton2.whenPressed(m_stagingToTop,false);
    
    //gbutton3.whenPressed(m_rotation);
    driverbutton6.whenPressed(m_extendIntake);
    driverbutton7.whenPressed(m_retractedIntake);
    rightTriggButton.whileHeld(m_manualLaunch);
    xButton.toggleWhenPressed(m_stagingToTop);

    startButton.toggleWhenActive(m_acquireTarget);
    //backButton.whenPressed(gyroReset);
    backButton.whenPressed(odometryReset);

    driverButton9.whileHeld(m_turnto90);

    //REMOVE IN COMPETITION CODE
    yButton.whenPressed(new InstantCommand(){
      @Override
      public void initialize(){
        CommandScheduler.getInstance().schedule(new Slalom(m_drivetrain));
      }
    });

    bButton.whenPressed(new InstantCommand(){
      @Override
      public void initialize(){
        Pose2d position = m_drivetrain.getRobotPose();
        Translation2d transform = position.getTranslation();
        System.out.println(String.format("Logged || x: %.3f || y: %.3f || rotation: %.1f", transform.getX(),transform.getY(), position.getRotation().getDegrees()));
      }
    });

  }

  void addAutonomous(){
    chooser.setDefaultOption("no autonomous", new PrintCommand("no autonomous selected"));
    Shuffleboard.getTab("Game screen").add(chooser);
  }

 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return chooser.getSelected();

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
