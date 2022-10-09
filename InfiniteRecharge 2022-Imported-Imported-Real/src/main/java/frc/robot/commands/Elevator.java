// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*AUTHORS NOTE: hey, one of the few things i didnt code this year! its a build season miracle! (or showing of larger
failures on my part considering this was one of the few commands that originally worked yet me still coding most of the robot)

I can mostly document this, but, if you got questions on how this works, direct them to Woogy. He can be found on discord at 
wg#7321
 */

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer; //why did we import a timer? i know why. week 6 north star we tried a weird timer thing to fix when 
//the flywheel was recovering slower. it didn't work. what did work was the timing code. check github to see it if you ever need to use a timer
//it'll be like the day one north star commit
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;

public class Elevator extends CommandBase {
  ElevatorSub m_elevatorSub;

  FlyWheel_Velocity m_FlyWheelVelocity = new FlyWheel_Velocity();
  public Elevator(ElevatorSub b) {
    m_elevatorSub = b; //yea should've been more descriptive than "b". maybe use elevatorSub
    addRequirements(m_elevatorSub); //adding reqs! only one command can run on the elevator sub at once
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

      m_elevatorSub.BackElevatorUp(.83); //.9 these speeds maximize the recovery time but are also crucial for getting the ball up to speed
      m_elevatorSub.FrontElevatorUp(.3);//.38 us having to do this is build teams fault (not rlly tho lol i only jest)
      //don't know why .38 is so specific

 


    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSub.ElevatorStop(); //stops the elevator
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
