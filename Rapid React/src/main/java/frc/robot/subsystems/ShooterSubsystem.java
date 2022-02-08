// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
// Creates a BangBangController
BangBangController BangBang = new BangBangController();
private final WPI_TalonFX falconShooter = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID); 
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    setCoast();
  }
  
  public void ShootBangBang() {

  }

  public void setCoast() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconShooter.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconShooter.setNeutralMode(NeutralMode.Brake);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
