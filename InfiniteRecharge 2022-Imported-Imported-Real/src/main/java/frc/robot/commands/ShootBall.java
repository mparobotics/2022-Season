/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LightMode;

public class ShootBall extends CommandBase {

  private ShooterSubsystem m_shootSub;
  public double m_yspeed;
  private Double m_setpoint;
  /**
   * Creates a new ShootBall.
   */
  public ShootBall(ShooterSubsystem shootSub, double y_speed) {
    m_shootSub = shootSub;
    m_yspeed = y_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shootSub);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_shootSub.setSetpoint(ShooterConstants.SHOOTER_SETPOINT);
    //m_shootSub.shootPIControl();
    RobotContainer.shooting = true;
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shootSub.shootPIControl();

   
   m_shootSub.ShootBangBang(m_yspeed);
    //SmartDashboard.putNumber("Shooter Temp", m_shootSub.getMotorTemp());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.stopShooter();
    ShooterSubsystem.adjustHood(0);
    RobotContainer.shooting = false;
    Limelight.setLedMode(LightMode.eOff);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
