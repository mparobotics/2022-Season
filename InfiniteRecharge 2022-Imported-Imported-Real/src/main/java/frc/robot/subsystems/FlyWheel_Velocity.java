// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FlyWheel_Velocity extends SubsystemBase {

  static WPI_TalonFX _talon = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID, "rio");
 
  public double speedINeed = 4000;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final Alliance allianceColor = DriverStation.getAlliance();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


  /** Creates a new FlyWheel_Velocity. */
  public FlyWheel_Velocity() {

    _talon.configFactoryDefault();
		_talon.setNeutralMode(NeutralMode.Coast);
		/* Config neutral deadband to be the smallest possible */
		_talon.configNeutralDeadband(0.001);


		/* Config sensor used for Primary PID [Velocity] */
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 30);
											
    _talon.setInverted(true);  
		/* Config the peak and nominal outputs */
		_talon.configNominalOutputForward(0, 30);
		_talon.configNominalOutputReverse(0, 30);
		_talon.configPeakOutputForward(1, 30);
		_talon.configPeakOutputReverse(-1, 30);

		/* Config the Velocity closed loop gains in slot0 */

		_talon.config_kF(0, 1023.0/20660.0, 30);
		_talon.config_kP(0, 1.331, 30); //0, 2, 30 .2 was working
		_talon.config_kI(0, 0.001 / 95, 30);
		_talon.config_kD(0,  5, 30); //0, 6, 30
    //SmartDashboard.putNumber("distance sim", 3);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", _talon.getSelectedSensorVelocity());
  }

  public  void my_Flywheel_Velocity(double setpoint){
    //new TurretAutoAlign(new TurretSubsystem());
    /**
			 * Convert 6200 RPM to units / 100ms.
			 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
    //double targetVelocity_UnitsPer100ms = setpoint * 6200.0 * 2048.0 / 600.0;
    speedINeed = setpoint;
    double targetVelocity_UnitsPer100ms = setpoint; //2000 * 2048 / 600
			/* 2000 RPM in either direction */
			_talon.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
      SmartDashboard.putNumber("Flywheel Speed Needed", setpoint);
      
  }



  public void myFlyWheel_PercentOut(double setpoint){
    _talon.set(TalonFXControlMode.PercentOutput, setpoint);
  }

  public boolean canIShoot(){
    
    if (_talon.getSelectedSensorVelocity() > (speedINeed - 50)){
    return true;}

    else {return false;}
    
  }  

  public boolean GetColor()
  {
    Color detectedColor = m_colorSensor.getColor();
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Red", detectedColor.red);
    Boolean correctColor;
    if (allianceColor == Alliance.Red)
    {
        if (detectedColor.blue == 1){
          correctColor = false;
        }
          else {correctColor = true;}
            }
    else {if (allianceColor == Alliance.Blue)
      {
      if (detectedColor.blue == 1){
        correctColor = false;
      }
      else {correctColor = true;}
    }
    else {correctColor = true;}

  }

    //SmartDashboard.putBoolean("CorrectColor?", correctColor);
    return correctColor;
  }
}


	