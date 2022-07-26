// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*THIS IS BAD CODE. DO NOT CODE THIS INTO YOUR PROJECT
 * IT IS ONLY HERE BECAUSE AT THE BEGINNING OF THE SEASON
 * I WAS UNAWARE OF WHAT THE end() LOOP DID
 * JUST USE THAT.
 * PLEASE, PLEASE, PLEASE FOR THE LOVE OF GOD DO NOT MAKE A "NEUTRAL" COMMAND
 * THAT ONLY TRIGERS WHEN YOU RELEASE THE BUTTON
 * 
 *  please
 */


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSub;

public class ElevatorNeutral extends CommandBase {
  ElevatorSub m_elevatorSub;
  public ElevatorNeutral(ElevatorSub b) {
    m_elevatorSub = b; //be more descriptive than just "b". Please.
    addRequirements(m_elevatorSub); // adds requirements for elevatorsub only one subsystem can use one command
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSub.ElevatorStop(); //stops the elevator
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
