// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class FlyWheelVelocityRunReverse extends CommandBase {
  /** Creates a new FlyWheelVelocityRun. */
  
  private final FlyWheel_Velocity m_flyWheel_Velocity;
  public FlyWheelVelocityRunReverse(FlyWheel_Velocity f) {
  
     m_flyWheel_Velocity = f;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


double speedToGet = -1000;

    m_flyWheel_Velocity.my_Flywheel_Velocity(speedToGet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flyWheel_Velocity.myFlyWheel_PercentOut(0);
    Limelight.setLedMode(LightMode.eOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
