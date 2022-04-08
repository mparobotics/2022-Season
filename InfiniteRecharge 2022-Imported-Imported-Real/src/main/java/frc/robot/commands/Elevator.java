// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;

public class Elevator extends CommandBase {
  ElevatorSub m_elevatorSub;
  public Timer reverseDelay = new Timer();
  FlyWheel_Velocity m_FlyWheelVelocity = new FlyWheel_Velocity();
  public Elevator(ElevatorSub b) {
    m_elevatorSub = b;
    addRequirements(m_elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reverseDelay.reset();
    reverseDelay.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     if (m_FlyWheelVelocity.canIShoot()) {
      m_elevatorSub.BackElevatorUp(1);
      m_elevatorSub.FrontElevatorUp(.4);
      reverseDelay.reset();
     }
     else{
      if (reverseDelay.get() == 0){
        reverseDelay.start();
      }
      
      if (reverseDelay.get() < .5)
     {
      m_elevatorSub.BackElevatorUp(-1);
      m_elevatorSub.FrontElevatorUp(-.4);
     }

     else {m_elevatorSub.ElevatorStop();}
    }


    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSub.ElevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
