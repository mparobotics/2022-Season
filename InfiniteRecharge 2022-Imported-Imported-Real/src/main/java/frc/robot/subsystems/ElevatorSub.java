// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.IntakeConstants;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new Intake. */
  static WPI_TalonSRX frontElev;
  static WPI_TalonSRX backElev;
  static CANSparkMax topElev;
  DigitalInput linebreak;
  MotorControllerGroup elevatorMotors;

  public ElevatorSub() {
    frontElev = new WPI_TalonSRX(ElevatorConstants.FRONT_ELEVATOR_ID);
    frontElev.setInverted(false);
    backElev = new WPI_TalonSRX(ElevatorConstants.BACK_ELEVATOR_ID);
    backElev.setInverted(false);
    linebreak = new DigitalInput(ElevatorConstants.LINEBREAK_RECIVER_ID);
    //topElev = new CANSparkMax(ElevatorConstants.TOP_ELEVATOR_ID, MotorType.kBrushless);

    //elevatorMotors = new MotorControllerGroup(frontElev, backElev);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void BackElevatorUp(double speed)
  {
    backElev.set(speed);
  }

  public void FrontElevatorUp(double speed)
  {
    frontElev.set(speed);
  }
  

    //topElev.set(speed);
  

  public void ElevatorStop()
  {
    frontElev.set(0);
    backElev.set(0);
    //topElev.set(0);
  }


  public boolean lineBreakBroken() {
    return linebreak.get(); 
  } 
}
