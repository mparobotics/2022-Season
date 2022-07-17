// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public DefaultDriveCommand(DriveSubsystem driveSubsystem,
  DoubleSupplier translationXSupplier,
  DoubleSupplier translationYSupplier,
  DoubleSupplier rotationSupplier) 
  {
  this.m_driveSubsystem = driveSubsystem;
  this.m_translationXSupplier = translationXSupplier;
  this.m_translationYSupplier = translationYSupplier;
  this.m_rotationSupplier = rotationSupplier;

  addRequirements(driveSubsystem);
}



/** Creates a new DefaultDriveCommand. */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
            m_driveSubsystem.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                      m_translationXSupplier.getAsDouble(),
                      m_translationYSupplier.getAsDouble(),
                      m_rotationSupplier.getAsDouble(),
                      m_driveSubsystem.getGyroscopeRotation()
              )
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
