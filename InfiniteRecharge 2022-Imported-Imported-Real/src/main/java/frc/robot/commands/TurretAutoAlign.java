// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class TurretAutoAlign extends CommandBase {
  /** Creates a new TurretAutoAlign. */
  TurretSubsystem turretSubsystem;
  public TurretAutoAlign() {
    //this needs requirements to be added
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn); //turns on shining death laser
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  Limelight.setLedMode(LightMode.eOn);  //turns on death laser again in case once wasn't enough
  RobotContainer.turretSubsystem.autoTurn(); //goes to autoTurn in turret subsystem
  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff); //saves our eyes from green death laser
    RobotContainer.turretSubsystem.turnTurret(0); //stops turning turret
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
