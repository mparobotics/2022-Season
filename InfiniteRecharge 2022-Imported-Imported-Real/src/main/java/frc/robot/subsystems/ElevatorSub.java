// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.IntakeConstants;

public class ElevatorSub extends SubsystemBase {
  /** Creates a new Intake. */
  static WPI_TalonSRX frontElev;
  static WPI_TalonSRX backElev;

  
  //DigitalInput linebreak = new DigitalInput(ElevatorConstants.LINEBREAK_RECIVER_ID);

  

  public ElevatorSub() {
    frontElev = new WPI_TalonSRX(ElevatorConstants.FRONT_ELEVATOR_ID);
    frontElev.setInverted(false);
    backElev = new WPI_TalonSRX(ElevatorConstants.BACK_ELEVATOR_ID);
    backElev.setInverted(false);
    

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

  public void ElevateBall(double speed)
  {
    frontElev.set(speed);
    backElev.set(speed);
  }



    //topElev.set(speed);
  

  public void ElevatorStop()
  {
    frontElev.set(0);
    backElev.set(0);
    //topElev.set(0);
  }


  /*public boolean lineBreakBroken() {
    return linebreak.get(); 
    
  } */
}
