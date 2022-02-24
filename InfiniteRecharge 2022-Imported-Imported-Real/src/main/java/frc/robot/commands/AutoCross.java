/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCross extends CommandBase {
  private DriveSubsystem m_driveSub;
  /**
   * Creates a new AutoDriveToWall.
   */
  public AutoCross() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.setDriveSpeed_Tank(-.5, -.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.setDriveSpeed_Tank(-.5, -.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.setDriveSpeed_Tank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.getAvgPosition() <= (-4 / DriveConstants.TIC_FT);
  }
}
