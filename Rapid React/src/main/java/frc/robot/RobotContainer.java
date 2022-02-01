// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LedRainbow;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.commands.TurretNeutral;
import frc.robot.commands.TurretTurnLeft;
import frc.robot.commands.TurretTurnRight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public DriveSubsystem driveSub = new DriveSubsystem();
  

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //Initializing Controllers
  public static XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  private XboxController helms = new XboxController(OIConstants.HELMS_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSub.setDefaultCommand(new ArcadeDrive (driveSub));
  }





  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(helms, Button.kA.value).whenHeld(new TurretAutoAlign());
    new JoystickButton(helms, Button.kLeftBumper.value).whenHeld(new TurretTurnLeft());
    new JoystickButton(helms, Button.kRightBumper.value).whenHeld(new TurretTurnRight());
    
    //neutralizes turret
    new JoystickButton(helms, Button.kA.value).whenReleased(new TurretNeutral());
    new JoystickButton(helms, Button.kLeftBumper.value).whenReleased(new TurretNeutral());
    new JoystickButton(helms, Button.kRightBumper.value).whenReleased(new TurretNeutral());
    
    //rainbows leds
    new JoystickButton(helms, Button.kB.value).whenHeld(new LedRainbow());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; //Todo ask mikey why annie did this?
  }
}
