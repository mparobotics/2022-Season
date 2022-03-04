// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheel_Velocity;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;


public class FlyWheelVelocityRun extends CommandBase {
  /** Creates a new FlyWheelVelocityRun. */
  
  private final FlyWheel_Velocity m_flywWheel_Velocity;
  public FlyWheelVelocityRun(FlyWheel_Velocity f) {
  
     m_flywWheel_Velocity = f;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double d = TurretSubsystem.getDistance();
    double speedToGet;
    d = SmartDashboard.getNumber("distance sim", 3);

    if (d < 7)
    {
      speedToGet = 1076 * d + 5970; 
    }
    else {speedToGet = 4500;}

    m_flywWheel_Velocity.my_Flywheel_Velocity(speedToGet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywWheel_Velocity.myFlyWheel_PercentOut(0);
    Limelight.setLedMode(LightMode.eOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
