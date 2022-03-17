// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class AutoDriveSubsystem extends SubsystemBase {
  /** Creates a new AutoDriveSubsystem. */
  private final static WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
  private final static WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final static WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final static WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private final static MotorControllerGroup SCG_R = new MotorControllerGroup(falconFR, falconBR); 
  private final static MotorControllerGroup SCG_L = new MotorControllerGroup(falconFL, falconBL); 

  private final static DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

  static final byte navx_rate = 127;
  AHRS navx = new AHRS(SPI.Port.kMXP, navx_rate);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());

  


  public AutoDriveSubsystem() {}

  @Override
  public void periodic() {
     // Update the odometry in the periodic block
     //m_odometry.update(
      //navx.getRotation2d(), falconFL.getSelectedSensorPosition(), falconFR.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(falconFL.getSelectedSensorPosition(), falconFR.getSelectedSensorPosition());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void resetEncoders() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
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
