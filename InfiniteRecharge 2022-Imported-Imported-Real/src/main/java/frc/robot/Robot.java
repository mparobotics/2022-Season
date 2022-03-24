/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCross;


import frc.robot.commands.Elevator;
import frc.robot.commands.FlyWheelVelocityRun;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.IntakeSub;

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
  private ElevatorSub m_ElevatorSub;
  private RobotContainer m_robotContainer;
  private DriveSubsystem m_DriveSubsystem;

  private FlyWheel_Velocity m_flywheelVelocity;
  private IntakeSub intakeSub = new IntakeSub();
  NetworkTable table;
  //private AutoIntakeDrop autoIntakeDrop;
  private AutoCross autoCross;
  private Elevator autoElevator;
  private SequentialCommandGroup ShootAndCross;
  private Intake intake = new Intake(intakeSub);
  private TurretAutoAlign turretAutoAlign = new TurretAutoAlign();
  private SequentialCommandGroup TrajTest;
  private FlyWheelVelocityRun spinFlywheel = new FlyWheelVelocityRun(new FlyWheel_Velocity()); 
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    m_robotContainer = new RobotContainer();
    m_DriveSubsystem = new DriveSubsystem();
    m_flywheelVelocity = new FlyWheel_Velocity();
 
    m_ElevatorSub = new ElevatorSub();
    Limelight.setLedMode(LightMode.eOff);
    autoElevator = new Elevator(m_ElevatorSub);
    autoCross = new AutoCross(m_robotContainer.driveSub);

    //ShootAndCross = new SequentialCommandGroup(autoIntakeDrop, autoCross, autoShoot);
    ShootAndCross = new SequentialCommandGroup(new IntakeDrop(intakeSub).withTimeout(2), 
                      autoElevator.withTimeout(6),
                        autoCross.withTimeout(3) );



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
    //RobotContainer.shooterSub.encoderReset();
    Limelight.setLedMode(LightMode.eOn); //TODO test
    //ShootAndCross.schedule();
    //autoCross.schedule();
    m_DriveSubsystem.zeroHeading();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    spinFlywheel.schedule();
    turretAutoAlign.schedule();
    intake.schedule();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("angle", m_DriveSubsystem.navx.getAngle());
    SmartDashboard.putNumber("rate", m_DriveSubsystem.navx.getRate());
    SmartDashboard.putString("heading", m_DriveSubsystem.getHeading().toString());




  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    //IntakeSub.IntakeDropStop();
    spinFlywheel.cancel();
    turretAutoAlign.cancel();
    intake.cancel();
    m_ElevatorSub.ElevatorStop();
    m_flywheelVelocity.my_Flywheel_Velocity(0);
    DriveSubsystem.stopRobot();
    m_DriveSubsystem.encoderReset();
    //RobotContainer.ShooterSub.encoderReset();
    //if (m_autonomousCommand != null) {
      //m_autonomousCommand.cancel();
    //}
    

    //set Limelight leds off at start of teleop
    Limelight.setLedMode(LightMode.eOff); //TODO test
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
