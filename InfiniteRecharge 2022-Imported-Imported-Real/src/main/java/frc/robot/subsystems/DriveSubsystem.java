/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

/**
 * this subsystem sets up and directly manipulates everything on the drive train
 */
public class DriveSubsystem extends SubsystemBase {

  //declaring and intializing drive motor controllers and assciated configuration objects
  private final static WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
  private final static WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final static WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final static WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private final static MotorControllerGroup SCG_R = new MotorControllerGroup(falconFR, falconBR); 
  private final static MotorControllerGroup SCG_L = new MotorControllerGroup(falconFL, falconBL); 

  private final static DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

  private final Field2d m_field = new Field2d();

  static final byte navx_rate = 127;
  public AHRS navx = new AHRS(SPI.Port.kMXP, navx_rate);

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  


  private static double error;
 

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    setBrake();
    //setting ramp
    falconFR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconFL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconBL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconBR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    //Drive Base Code
    falconBR.follow(falconFR); //talonBR follows TalonFR
    falconBL.follow(falconFL); //talonBL follows TalonFR 

    falconFR.setInverted(false); //set to invert falconFR.. CW/CCW.. Green = forward (motor led)
    falconBR.setInverted(false);
    falconFL.setInverted(true);
    falconBL.setInverted(true);
    
    //matches whatever falconFR is
    //falconFL.setInverted(true); //set to invert falconFL.. CW/CCW.. Green = foward (motor led)
    
    


    
    //Encoder Code Start
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Selecting Feedback Sensor
   
   encoderReset(); //TODO I wonder if this is the problem 
  }

  public void setCoast() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Coast);
    falconFL.setNeutralMode(NeutralMode.Coast);
    falconBR.setNeutralMode(NeutralMode.Coast);
    falconBL.setNeutralMode(NeutralMode.Coast);
  }
  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
    falconBR.setNeutralMode(NeutralMode.Brake);
    falconBL.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  setDriveSpeed_Arcade(-RobotContainer.xbox.getLeftY(), RobotContainer.xbox.getRightX()*.75);
  //SmartDashboard.putNumber("Left Encoder", falconFL.getSelectedSensorPosition());
  //SmartDashboard.putNumber("Right Encoder", falconFR.getSelectedSensorPosition());
  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  SmartDashboard.putData("Field", m_field);

  m_odometry.update(
 
    navx.getRotation2d(), 
    (((falconFL.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters),
    (((falconFR.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters)); 
    SmartDashboard.putNumber("left troubleshooting", (((falconFL.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters));
    SmartDashboard.putNumber("right troubleshooting", (((falconFR.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters));
    //10.91 is the gear ratio, 2piRadius is circumfrence of the wheel, divide by 60 to get from min to sec 
    //divide by MPArunits ratio (mpu) to convert from MPAror Units to Meters
    

  var translation = m_odometry.getPoseMeters().getTranslation();
  m_xEntry.setNumber(translation.getX());
  m_yEntry.setNumber(translation.getY());  
  
  m_field.setRobotPose(m_odometry.getPoseMeters());
  SmartDashboard.putNumber("right encoder", falconFR.getSelectedSensorPosition());
  SmartDashboard.putNumber("left encoder", falconFL.getSelectedSensorPosition());

}

  private static double driveTrainP() {
    error = falconFL.getSelectedSensorPosition() - falconFR.getSelectedSensorPosition();
    //integral += error*.02;
    return DriveConstants.DRIVE_P*error;
  }

  public static void driveStraight(double xSpeed) {
    drive.arcadeDrive(xSpeed, -driveTrainP());
  }

  /**
   * sets the speed of the drive train with arcade controls
   * @param xSpeed
   * @param zRotation
   */
  public void setDriveSpeed_Arcade(double xSpeed, double zRotation) {
    xSpeed = xSpeed * .95;
    zRotation = zRotation * .85;
  /*  if (RobotContainer.helms.getRawButton(9) == true) {
      xSpeed = xSpeed / 2;
    }
    if (RobotContainer.helms.getRawButton(10) == true) {
      zRotation = zRotation / 2;
    }*/
    if (Math.abs(xSpeed) < .1) {xSpeed = 0;}//deadzones
    if (Math.abs(zRotation) < .1) {zRotation = 0;}//deadzones
    if (zRotation == 0 )
      driveStraight(xSpeed);
    
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * set the speed of the drive train with tank controls (WIP)
   * @param lSpeed
   * @param rSpeed
   */
  public static void setDriveSpeed_Tank(double lSpeed, double rSpeed) {
    drive.tankDrive(lSpeed, rSpeed);
  }

  /**
   * stops the drive train
   */
  public static void stopRobot() {
    falconFR.set(ControlMode.PercentOutput, 0);
    falconFL.set(ControlMode.PercentOutput, 0);
  }

  /**
   * prints encoder values to the smart dashboard
   */
  public void printEncoderValues() {
    SmartDashboard.putNumber("FR pos", falconFR.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR pos", falconBR.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL pos", falconFL.getSelectedSensorPosition());
    SmartDashboard.putNumber("BL pos", falconBL.getSelectedSensorPosition());
  }

  public static double getAvgPosition() {
    return (falconFR.getSelectedSensorPosition() + falconBR.getSelectedSensorPosition()) / 2;
  }

  public void encoderReset() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(falconFL.getSelectedSensorPosition() * DriveConstants.Conversion, falconFR.getSelectedSensorPosition() * DriveConstants.Conversion);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SCG_L.setVoltage(leftVolts);
    SCG_R.setVoltage(rightVolts);
    drive.feed();
  }

  public double getAverageEncoderDistance() {
    return (falconFL.getSelectedSensorPosition() + falconFR.getSelectedSensorPosition()) / 2.0;
  }

    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public double getTurnRate() {
    return -navx.getRate();
  }
}




