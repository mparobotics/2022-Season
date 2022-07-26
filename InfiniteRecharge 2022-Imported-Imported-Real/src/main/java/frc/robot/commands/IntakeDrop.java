// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSub;

public class IntakeDrop extends CommandBase {
  /** Creates a new IntakeDrop. */
  IntakeSub intakeSub;
  public IntakeDrop(IntakeSub i) {
    intakeSub = i;
    addRequirements(intakeSub);
    //ive explained command reqs enough continue on with your day already
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        



    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSub.IntakeDrop(); //drops the intake using motor

    //SmartDashboard.putBoolean("Is intake down?", IntakeSub.IntakeIsDown);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSub.IntakeDropStop(); // stops dropping intake
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
