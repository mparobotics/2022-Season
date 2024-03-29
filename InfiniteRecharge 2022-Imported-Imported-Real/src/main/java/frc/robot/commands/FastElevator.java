// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*Its the elevator but fast
 * YAY
 * the code has changed in no meaningful way
 * maybe i should have made it so you could feed the command speeds as a requirement
 * instead of having like, 5 elevaytor commands
 * hindsight is 20/20
 * note to future me: do that even though you don't think you will need more than 2 commands
 * you will.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.FlyWheel_Velocity;

public class FastElevator extends CommandBase {
  ElevatorSub m_elevatorSub;
  FlyWheel_Velocity m_FlyWheelVelocity = new FlyWheel_Velocity();
  public FastElevator(ElevatorSub b) {
    m_elevatorSub = b;
    addRequirements(m_elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     
      m_elevatorSub.BackElevatorUp(1);
      m_elevatorSub.FrontElevatorUp(.7);

    
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
