// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSub;

public class Intake extends CommandBase {
  IntakeSub intakeSub;
  public Intake(IntakeSub i) {
    intakeSub = i; //be more descriptive than "i" in the future
    addRequirements(intakeSub); //only one command per subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSub.intakeBall(IntakeConstants.INTAKE_SPEED); // sets intake to spin at intake speed defined in constants
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //should set the intake to 0 in here instead of mapping a seperate when released command
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
