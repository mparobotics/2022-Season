/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int deviceID = 50; //setting CAN ID here
  private CANSparkMax m_motor;
  private CANEncoder m_encoder;
  private final WPI_TalonSRX RL1 = new WPI_TalonSRX(20);
  private final WPI_TalonSRX RL2 = new WPI_TalonSRX(2);
  PowerDistribution examplePD = new PowerDistribution();
  private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  private NetworkTableEntry maxSpeed = 
    tab.add("Max Speed", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();    


  /**
   * This function is run when the robot is first started up and should be
   * used for any initial ization code.
   */
  @Override
  public void robotInit() {

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless); //setting weather its a brushed or non brushed motor
    RL2.follow(RL1);
    RL1.setInverted(false);
    RL2.setInverted(InvertType.FollowMaster);
    m_encoder = m_motor.getEncoder();
    }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double max = maxSpeed.getDouble(0);
    //robotDrive.tankDrive(left * max, right * max);
    /**
     * Encoder position is read from a CANEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Neo Encoder Position", m_encoder.getPosition());

    /**
     * Encoder velocity is read from a CANEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Neo Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Neo Encoder Vel Graph", m_encoder.getVelocity());

    SmartDashboard.putNumber("Neo Tempature", m_motor.getMotorTemperature());
    SmartDashboard.putNumber("NeoCurrent", examplePD.getCurrent(9));
    //SmartDashboard.putNumber("SRX1Current", examplePD.getCurrent(8));
    //SmartDashboard.putNumber("SRX2Current", examplePD.getCurrent(7));
    //SmartDashboard.putNumber("Total Power in watts", examplePD.getTotalPower()); //bus voltage x current in watts
    //SmartDashboard.putNumber("Total Power in Joules", examplePD.getTotalEnergy());

    //SmartDashboard.putNumber("RL VEL Graph", RL1.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("RL VEL Graph", RL1.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("RL output Pos", RL1.getSelectedSensorPosition());
    //SmartDashboard.putNumber("RL1 out%", RL1.getMotorOutputPercent());
    //SmartDashboard.putNumber("RL2 out%", RL2.getMotorOutputPercent());

    m_motor.set(max);
    //RL1.set(max);
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
