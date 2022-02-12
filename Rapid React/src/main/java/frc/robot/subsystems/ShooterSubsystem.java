// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
// Creates a BangBangController
BangBangController BangBang = new BangBangController(); //instantiartes bangbang motor controller
private final WPI_TalonFX falconShooter = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID); 
SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
public double setpoint = 0.0; 
public double angle;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    setCoast();
  }
  
  public void ShootBangBang() {
    falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorPosition(), (getSetpoint()) + 0.9 * feedforward.calculate(setpoint)));

    //bang bang based on size of d from the hub

  }

  public double getSetpoint () {
    double d = TurretSubsystem.getDistance();
    
    ///math
    
    return setpoint;
  }

  public double getAngle(){
    double d = TurretSubsystem.getDistance();
    
    //math

    return angle;

  }

  public void alignHood(double angle) {


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
