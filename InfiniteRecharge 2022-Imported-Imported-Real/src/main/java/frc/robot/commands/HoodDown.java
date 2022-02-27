// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class HoodDown extends CommandBase {
  /** Creates a new TurretTurnRight. */
  ShooterSubsystem shooterSub;
  public HoodDown() {
   
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (TurretSubsystem.m_encoder.getPosition() < -ShooterConstants.max_turret_rotation)
    //{
     // new TurretCenter();
    //}
    
    //else
     {RobotContainer.shooterSub.adjustHood(.5);} //todo test speed + invert
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSub.adjustHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
