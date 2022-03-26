/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;

public class AutoCross extends CommandBase {
  private DriveSubsystem m_driveSub;
  private TurretSubsystem m_turretSubsystem;
  double leftEncoderStart;
  /**
   * Creates a new AutoDriveToWall.
   */
  public AutoCross(DriveSubsystem driveSub) {
    m_driveSub = driveSub;

    addRequirements(m_driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftEncoderStart = m_driveSub.getLeftEncoder();

    Limelight.setLedMode(LightMode.eOn);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    IntakeSub.intakeBall(IntakeConstants.INTAKE_SPEED);
    
    m_driveSub.tankDriveVolts(12, 12);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.setDriveSpeed_Tank(0, 0);
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return m_driveSub.getLeftEncoder() > 80000 + leftEncoderStart;//JUST AN FYI + IS BACKWARDS HERE AND - IS FORWARD
    

  }
      

}

