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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final static WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
  private final static WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final static WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final static WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 
  
  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private final static MotorControllerGroup SCG_R = new MotorControllerGroup(falconFR, falconBR); 
  private final static MotorControllerGroup SCG_L = new MotorControllerGroup(falconFL, falconBL); 

  private final static DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);
  
  static final byte navx_rate = 127;
  public AHRS navx = new AHRS(SPI.Port.kMXP, navx_rate);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private static double error;

  /**data Logging define logs */
  DoubleLogEntry encoderLogRawR;
  DoubleLogEntry encoderLogRawL;
  DoubleLogEntry encoderLogMetersR;
  DoubleLogEntry encoderLogMetersL;
  DoubleLogEntry poseLog; //this
  DoubleLogEntry odomentryLog;
  DoubleLogEntry headingLogR2d; //this
  DoubleLogEntry angleLog;
  StringLogEntry rotation2dlog; //this
  //FLoatLogEntry myFloatLog;
  //IntegerLogEntry myIntegerLogEntry;
  //StringLogEntry myStringLog;
    

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    setBrake();
    //setting ramp
    falconFR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconFR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconFL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconFL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconBL.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconBL.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    falconBR.configOpenloopRamp(0.4); // 0.5 seconds from neutral to full output (during open-loop control)
    //falconBR.configClosedloopRamp(0.1); // 0 disables ramping (during closed-loop control)

    //Drive Base Code
    falconBR.follow(falconFR); //talonBR follows TalonFR
    falconBL.follow(falconFL); //talonBL follows TalonFR 

    falconFR.setInverted(false); //set to invert falconFR.. CW/CCW.. Green = forward (motor led)
    falconFL.setInverted(true);

    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Selecting Feedback Sensor

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d());


    DataLog log = DataLogManager.getLog();
    encoderLogRawR = new DoubleLogEntry(log, "/auto/encodeRawR");
    encoderLogRawL = new DoubleLogEntry(log, "/auto/encodeRawL");
    encoderLogMetersR = new DoubleLogEntry(log, "/auto/encodeMR");
    encoderLogMetersL = new DoubleLogEntry(log, "/auto/encodeML");
    headingLogR2d = new DoubleLogEntry(log, "/auto/heading"); 
    angleLog = new DoubleLogEntry(log, "/auto/angle");
 
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      navx.getRotation2d(), 
      (((falconFL.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters),
      (((falconFR.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters));
    dataLogTrajectory();
    SmartDashboard.putNumber("angle", navx.getAngle());
    SmartDashboard.putNumber("rate", navx.getRate());
  }

  private static double driveTrainP() {
    error = falconFL.getSelectedSensorPosition() - falconFR.getSelectedSensorPosition();
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

  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double frontLeftSpeed = falconFL.getSelectedSensorVelocity() * (10.0/2048) * Constants.DriveConstants.WheelCircumferenceMeters;
    double frontRightSpeed = falconFR.getSelectedSensorVelocity() * (10.0/2048) * Constants.DriveConstants.WheelCircumferenceMeters;

    return new DifferentialDriveWheelSpeeds(frontLeftSpeed, frontRightSpeed);
    
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SCG_L.setVoltage(leftVolts);
    SCG_R.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (falconFL.getSelectedSensorPosition() + falconFR.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return falconFL.getSelectedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return falconFR.getSelectedSensorPosition();
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
    //return m_gyro.getRotation2d().getDegrees();
    return Rotation2d.fromDegrees(-navx.getAngle());
  }

  public void dataLogTrajectory() {
    DataLogManager.start();
    encoderLogRawR.append(falconFR.getSelectedSensorPosition());
    encoderLogRawL.append(falconFL.getSelectedSensorPosition());
    encoderLogMetersR.append((((falconFR.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters));
    encoderLogMetersL.append((((falconFL.getSelectedSensorPosition() / Constants.DriveConstants.EncoderTPR) / Constants.DriveConstants.GearRatio) * Constants.DriveConstants.WheelCircumferenceMeters));
    headingLogR2d.append(-navx.getAngle());
    angleLog.append(getTurnRate());
  }

  //TODO Something to Try
  /*
  public double getHeading() {
    return navx.getYaw();
  }
  */

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navx.getRate();
  }//TODO Check
}