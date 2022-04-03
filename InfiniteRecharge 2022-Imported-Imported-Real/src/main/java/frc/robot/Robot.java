/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCross;
import frc.robot.commands.AutoReturn;
import frc.robot.commands.BallShoot;
import frc.robot.commands.Elevator;
import frc.robot.commands.FlyWheelVelocityRun;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.NullCommand;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.IntakeSub;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.I2C;


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

  private AutoCross autoCross;
  private AutoReturn autoReturn;
  private Elevator autoElevator;
  private Command ShootAndCross;
  private ParallelCommandGroup ParallelTwoBall;
  private Intake intake = new Intake(intakeSub);
  private TurretAutoAlign turretAutoAlign = new TurretAutoAlign();
  private SequentialCommandGroup TrajTest;
  private FlyWheelVelocityRun spinFlywheel = new FlyWheelVelocityRun(new FlyWheel_Velocity()); 
  private IntakeDrop intakeDrop =  new IntakeDrop(intakeSub);
  private WaitCommand waitCommand = new WaitCommand(2);
  private NullCommand nullCommand = new NullCommand();
  private BallShoot ballShoot = new BallShoot();
  
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  
   /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  
  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Specify the colors you are trying to match
   */
  private final Color kBlueTarget = new Color(0.239, 0.506, 0.255);
  private final Color kRedTarget = new Color(0.293, 0.499, 0.207);
  

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
    //Limelight.setLedMode(LightMode.eOff);
    autoElevator = new Elevator(m_ElevatorSub);
    autoCross = new AutoCross(m_robotContainer.driveSub);
    autoReturn = new AutoReturn(m_robotContainer.driveSub);

    
    


    //ShootAndCross = new SequentialCommandGroup(autoIntakeDrop, autoCross, autoShoot);
    ShootAndCross = new SequentialCommandGroup(
                        intakeDrop.withTimeout(1), autoCross, autoReturn, nullCommand.withTimeout(3), autoElevator.withTimeout(4));
      

    //ParallelTwoBall = new ParallelCommandGroup(new IntakeDrop(intakeSub).withTimeout(2), ShootAndCross, spinFlywheel, turretAutoAlign);

    //table = NetworkTableInstance.getDefault().getTable("limelight"); //Gets Table instance
    //table.getEntry("ledMode").setNumber(1); //sets limelight LEDS to "off"
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
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

        /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    double IR = m_colorSensor.getIR();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    /**
     * In addition to RGB IR values, the color sensor can also return an 
     * infrared proximity value. The chip contains an IR led which will emit
     * IR pulses and measure the intensity of the return. When an object is 
     * close the value of the proximity will be large (max 2047 with default
     * settings) and will approach zero when the object is far away.
     * 
     * Proximity can be used to roughly approximate the distance of an object
     * or provide a threshold for when an object is close enough to provide
     * accurate color values.
     */
    int proximity = m_colorSensor.getProximity();

    //SmartDashboard.putNumber("Proximity", proximity);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    //set limelight off when robot is disabled
    Limelight.setLedMode(LightMode.eOn); //TODO test
    //table.getEntry("ledMode").setNumber(1);
    m_DriveSubsystem.zeroHeading();
    m_DriveSubsystem.encoderReset();
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("heading", m_DriveSubsystem.getHeading());
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
  
    DataLogManager.start();
    //set Limelight at auto start
    //m_DriveSubsystem.encoderReset();
    //m_DriveSubsystem.zeroHeading();
    //Limelight.setLedMode(LightMode.eOn); //TODO test
    //ShootAndCross.schedule();
    //ParallelTwoBall.schedule();
    
    //m_DriveSubsystem.zeroHeading();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //ballShoot.schedule();
   //intake.schedule();
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
    //SmartDashboard.putNumber("angle", m_DriveSubsystem.navx.getAngle());
    //SmartDashboard.putNumber("rate", m_DriveSubsystem.navx.getRate());
    //SmartDashboard.putString("heading", m_DriveSubsystem.getHeading().toString());

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
