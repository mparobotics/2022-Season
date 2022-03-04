// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurretAutoAlign;

public class FlyWheel_Velocity extends SubsystemBase {

  static WPI_TalonFX _talon = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID, "rio");



  /** Creates a new FlyWheel_Velocity. */
  public FlyWheel_Velocity() {

    _talon.configFactoryDefault();
		_talon.setNeutralMode(NeutralMode.Coast);
		/* Config neutral deadband to be the smallest possible */
		_talon.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 30);
											

		/* Config the peak and nominal outputs */
		_talon.configNominalOutputForward(0, 30);
		_talon.configNominalOutputReverse(0, 30);
		_talon.configPeakOutputForward(1, 30);
		_talon.configPeakOutputReverse(-1, 30);

		/* Config the Velocity closed loop gains in slot0 */

		_talon.config_kF(0, 1023.0/20660.0, 30);
		_talon.config_kP(0, .1, 30);
		_talon.config_kI(0, 0.001 / 75, 30);
		_talon.config_kD(0, 5, 30);
    //SmartDashboard.putNumber("distance sim", 3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void my_Flywheel_Velocity(double setpoint){
    //new TurretAutoAlign(new TurretSubsystem());
    /**
			 * Convert 6200 RPM to units / 100ms.
			 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
    //double targetVelocity_UnitsPer100ms = setpoint * 6200.0 * 2048.0 / 600.0;
    double targetVelocity_UnitsPer100ms = setpoint; //2000 * 2048 / 600
			/* 2000 RPM in either direction */
			_talon.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
      SmartDashboard.putNumber("Flywheel Speed Needed", setpoint);
      SmartDashboard.putNumber("flywheel speed", _talon.getSelectedSensorVelocity());
      
  }

  public void myFlyWheel_PercentOut(double setpoint){
    _talon.set(TalonFXControlMode.PercentOutput, setpoint);
  }
}


	