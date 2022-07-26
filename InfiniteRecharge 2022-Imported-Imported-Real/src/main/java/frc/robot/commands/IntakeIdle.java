// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSub;

public class IntakeIdle extends CommandBase {
  IntakeSub intakeSub;
  public IntakeIdle(IntakeSub i) {
    intakeSub = i;
    addRequirements(intakeSub);
    //they're reqs. ok. go to a higher command if u still dont know what they are
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSub.intakeBall(.5); //always idles intake at .5 speed. this is called whenever a intake command ends in robotcontainer
    //it was called poorly. 
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
