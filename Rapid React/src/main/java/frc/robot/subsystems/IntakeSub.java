// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  /** Creates a new Intake. */
  static WPI_TalonSRX intake;

  public IntakeSub() {
    intake = new WPI_TalonSRX(IntakeConstants.INTAKE_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void intakeBall(double speed)
  {
    intake.set(speed);
  }

  public static void IntakeStop()
  {
    intake.set(0);
  }
}
