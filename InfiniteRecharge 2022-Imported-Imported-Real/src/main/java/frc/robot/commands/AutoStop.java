// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStop extends InstantCommand {
  private DriveSubsystem m_driveSub;
  public AutoStop(DriveSubsystem driveSub) {
    m_driveSub = driveSub;

    addRequirements(m_driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSub.tankDriveVolts(0.0, 0.0); 
    //instant command to stop driving i think?? dont know why its here or if we used it ever lol

  }
}
