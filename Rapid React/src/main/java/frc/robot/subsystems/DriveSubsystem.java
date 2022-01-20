// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class DriveSubsystem extends SubsystemBase {
  /** Defines and initializes motors using IDs from constants */
  private final WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
  private final WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  /** makes speed control groups, now called motor control groups. yay changing wording
   * for no good reason wasting an hour of my time, yay! */
  private final MotorControllerGroup MCG_R = new MotorControllerGroup(falconFR, falconBR); 
  private final MotorControllerGroup MCG_L = new MotorControllerGroup(falconFL, falconBL); 

  private final DifferentialDrive drive = new DifferentialDrive(MCG_L, MCG_R); //todo check right side inversion

  private double error;
  private double integral;

  public DriveSubsystem() {
    
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
  }
  //calculates shit so we can drive straight
  private double driveTrainP() {
    error = falconFL.getSelectedSensorPosition() - falconFR.getSelectedSensorPosition();
    //integral += error*.02;
    return DriveConstants.DRIVE_P*error;
  }
  //drives straight
  public void driveStraight(double xSpeed) {
    drive.arcadeDrive(xSpeed, -driveTrainP());
  }
  /**
   * sets the speed of the drive train with arcade controls ask Mikey what it does? If you see this remind me to ask them.
   * @param xSpeed
   * @param zRotation
   */
  
   //arcade drive, yay! discussion should we do tank this year? discussion we need to have
   public void setDriveSpeed_Arcade(double xSpeed, double zRotation) {
    if (zRotation == 0 )
      driveStraight(xSpeed);
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * set the speed of the drive train with tank controls (WIP) still not sure what this does -ian
   * @param lSpeed
   * @param rSpeed
   */
  //its tank drive! in case we want to use tank drive here is is! how fun! how riveting! is that how i speel riveting?
   public void setDriveSpeed_Tank(double lSpeed, double rSpeed) {
    drive.tankDrive(lSpeed, rSpeed);
  }

    /**
   * stops the drive train
   */
  public void stopRobot() {
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

  public void encoderReset() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
  }
}
