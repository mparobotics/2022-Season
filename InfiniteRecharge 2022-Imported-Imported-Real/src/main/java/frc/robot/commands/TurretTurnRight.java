// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTurnRight extends CommandBase {
  /** Creates a new TurretTurnRight. */
  TurretSubsystem turretSubsystem;
  public TurretTurnRight(TurretSubsystem t) {
      turretSubsystem = t;
      addRequirements(turretSubsystem);
      //more requirement adding fun
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (TurretSubsystem.m_encoder.getPosition() < -ShooterConstants.max_turret_rotation)
    {
      SmartDashboard.putBoolean("Turret Good", false);
    }*/
    
    //else
     RobotContainer.turretSubsystem.turnTurret(-ShooterConstants.TURRET_SPEED); // turns the turret to the right
     //SmartDashboard.putBoolean("Turret Good", true);
     //todo test speed 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //the turret should be coded to stop moving here
    //it isnt.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
