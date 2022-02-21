/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.Joystick; //TODO ButtonBoardTest

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDriveClassic;
import frc.robot.commands.Elevator;
import frc.robot.commands.ElevatorNeutral;
import frc.robot.commands.ElevatorReverse;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.commands.TurretCenter;
import frc.robot.commands.TurretNeutral;
import frc.robot.commands.TurretTurnLeft;
import frc.robot.commands.TurretTurnRight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * 
 * Contains subsystems, OI devices, and commands (runs commands)
 */
public class RobotContainer {
  // declaring and intializing subsystems(all) and commands (most commands don't need to be delclared/intialized here)
  public DriveSubsystem driveSub      = new DriveSubsystem();
  public IntakeSub intakeSub = new IntakeSub(); 
  public ElevatorSub elevatorsub = new ElevatorSub();
  public static ShooterSubsystem shooterSub = new ShooterSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  // declaring and intializing controller(s)
  private XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  private XboxController helms = new XboxController(OIConstants.HELMS_ID);
  //private Joystick buttonBoard = new Joystick(OIConstants.buttonBoard); //TODO ButtonBoardTEST

  public static boolean shooting = false;
   
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drive controls
    /*driveSub.setDefaultCommand(new ArcadeDriveTrigger(driveSub, 
                              () -> xbox.getLeftTriggerAxis(), 
                              () -> xbox.getRightTriggerAxis(),
                              () -> xbox.getLeftX()));*/
  
   driveSub.setDefaultCommand(new ArcadeDriveClassic(driveSub,
                              () -> xbox.getLeftY(),
                              () -> xbox.getRightX()*.75));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Main's Commands - Winch is right stick, 
    //new JoystickButton(xbox, Button.kBumperRight.value).whenHeld(new StartWinch(climberSub)); // Disabled - Climber Taken off
      new JoystickButton(helms, Button.kB.value).whenHeld(new ShootBall(shooterSub));
        //intake settings
        new JoystickButton(helms, Button.kX.value).whenHeld(new Intake(intakeSub));
        new JoystickButton(xbox, Button.kY.value).whenHeld(new IntakeReverse(intakeSub));
        //intake stop
        new JoystickButton(helms, Button.kX.value).whenReleased(new IntakeStop());
        new JoystickButton(xbox, Button.kY.value).whenReleased(new IntakeStop());

         //turret stuff
        new JoystickButton(xbox, Button.kB.value).whenHeld(new TurretAutoAlign(turretSubsystem));
        new JoystickButton(xbox, Button.kLeftBumper.value).whenHeld(new TurretTurnLeft(turretSubsystem));
        new JoystickButton(xbox, Button.kRightBumper.value).whenHeld(new TurretTurnRight(turretSubsystem));
        //new JoystickButton(helms, Button.kA.value).whenHeld(new TurretCenter());
    //neutralizes turret
        new JoystickButton(helms, Button.kB.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(xbox, Button.kLeftBumper.value).whenReleased(new TurretNeutral(turretSubsystem));
        new JoystickButton(xbox, Button.kRightBumper.value).whenReleased(new TurretNeutral(turretSubsystem));

    
        //intake dropdown
        //new JoystickButton(helms, Button.kX.value).whenHeld(new IntakeDrop());
       
        
        //Elevator settings
        new JoystickButton(helms, Button.kA.value).whenHeld(new Elevator(elevatorsub));
        new JoystickButton(xbox, Button.kY.value).whenHeld(new ElevatorReverse(elevatorsub));
        //elevator stop
        new JoystickButton(helms, Button.kA.value).whenReleased(new ElevatorNeutral(elevatorsub));
        new JoystickButton(xbox, Button.kY.value).whenReleased(new ElevatorNeutral(elevatorsub));
    


    //Helm's Commands
    //new JoystickButton(helms, Button.kBack.value).whenHeld(new ClimberUp(climberSub)); // Disabled - Climber Taken off
  

    

    //Button Board
    //new JoystickButton(buttonBoard, Button).whenHeld(command); //TODO FIGURE THIS OUT (BUTTONBOARDTEST)
  }

  public XboxController getController() {
    return xbox; 
  }

  public XboxController getHelms() {
    return helms;
  }

  //public Joystick getbuttonBoard() { //TODO BUTTONBOARDTEST
    //return buttonBoard;
  //}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * currently there is no auto command
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
