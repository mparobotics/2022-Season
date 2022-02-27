/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDriveClassic;
import frc.robot.commands.AutoCross;
import frc.robot.commands.AutoShootBall;
import frc.robot.commands.ElevatorNeutral;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSubsystem;


import edu.wpi.first.networktables.NetworkTable;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  NetworkTable table;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  private AutoCross autoCross;
  private AutoShootBall autoShoot;
  private SequentialCommandGroup ShootAndCross;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    m_robotContainer = new RobotContainer();
    Limelight.setLedMode(LightMode.eOff);
    autoShoot = new AutoShootBall();
    autoCross = new AutoCross();
    //ShootAndCross = new SequentialCommandGroup(autoCross, autoShoot) {
      
    //};
    

   

    //table = NetworkTableInstance.getDefault().getTable("limelight"); //Gets Table instance
    //table.getEntry("ledMode").setNumber(1); //sets limelight LEDS to "off"
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    //set limelight off when robot is disabled
    Limelight.setLedMode(LightMode.eOn); //TODO test
    //table.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void disabledPeriodic() {
  
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
  
    //set Limelight at auto start
    m_robotContainer.driveSub.encoderReset();
    RobotContainer.shooterSub.encoderReset();
    Limelight.setLedMode(LightMode.eOn); //TODO test
    
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.schedule();
    //}

    autoCross.schedule();
    //autoShoot.schedule();
    /**switch (autoChooser.getSelected().toString()) {
      case "Shœot lé bOl":
      default:
      autoShoot.schedule();
      break;
      case "cR√os lînë":
      autoCross.schedule();
      break;
      case "dO Nøthîng":
      break;
    } **/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    IntakeSub.IntakeDropStop();
    ElevatorSub.ElevatorStop();
    ShooterSubsystem.stopShooter();  
    DriveSubsystem.stopRobot();
    RobotContainer.shooterSub.encoderReset();
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.cancel();
    //}
    

    //set Limelight leds off at start of teleop
    Limelight.setLedMode(LightMode.eOn); //TODO test
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    //printing encoder values (testing)
    //m_robotContainer.driveSub.printEncoderValues();
  }
}
