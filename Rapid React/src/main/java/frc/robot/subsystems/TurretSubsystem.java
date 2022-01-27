// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  CANSparkMax m_motor = new CANSparkMax(ShooterConstants.NEO_TURRET_ID, MotorType.kBrushless); 
  //setting weather its a brushed or non brushed motor
  //defines the motor
  public TurretSubsystem() {
}
  public void turnTurret (double turnSpeed) {
    m_motor.set(turnSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}