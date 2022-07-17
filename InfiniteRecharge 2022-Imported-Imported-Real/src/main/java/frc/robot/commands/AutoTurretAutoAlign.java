// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class AutoTurretAutoAlign extends CommandBase {
  /** Creates a new TurretAutoAlign. */
  TurretSubsystem turretSubsystem;
  public AutoTurretAutoAlign() {
 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn); //turns on limelight
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  Limelight.setLedMode(LightMode.eOn); //turns on limelight again i guess, because turning it on once just isn't enough apparently
  RobotContainer.turretSubsystem.autoTurnAuto(); //goes to autoTurn in turret subsystem  
  /*future me – hey, i actually documented something in build season! thats what the commment above is for. I don't know what
  *autoTurnAuto does in the subsystem but i'll get around to documenting it if i don't give up in the mean time or get hit by a truck
  tomorrow or something. im assuming its for turning the turret in auto. why i would need a seperate command for that, i have no clue.
  */
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Limelight.setLedMode(LightMode.eOff); //turns off the limelight. savior of human's eyes everywhere.
    RobotContainer.turretSubsystem.turnTurret(0); //stops the turret spinning
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
