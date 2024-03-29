// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTurnLeft extends CommandBase {
  /** Creates a new TurretTurnLeft. */
  TurretSubsystem turretSubsystem;
  public TurretTurnLeft(TurretSubsystem t) {
      turretSubsystem = t;
      addRequirements(turretSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TurretSubsystem.m_encoder.getPosition() > ShooterConstants.max_turret_rotation)
    {
      new TurretCenter();
    }
    RobotContainer.turretSubsystem.turnTurret(.1); //to test speed plus invert
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
