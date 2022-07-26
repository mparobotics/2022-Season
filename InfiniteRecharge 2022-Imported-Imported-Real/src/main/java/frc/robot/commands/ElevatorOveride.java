// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*This was an overide for the elevator tied to the button box
 * It was during the timer thing we did at north star
 * It was in case the timer didn't work and was tied to the button box
 * We used it one match
 * at this point its just a clone of elevator.java
 * see that if u want documentation
 * this is copy pasted code, y'all don't need copy pasted documentation
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;

public class ElevatorOveride extends CommandBase {
  ElevatorSub m_elevatorSub;
  public ElevatorOveride(ElevatorSub b) {
    m_elevatorSub = b; 
    addRequirements(m_elevatorSub); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSub.BackElevatorUp(1); 
    m_elevatorSub.FrontElevatorUp(.4); 
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
