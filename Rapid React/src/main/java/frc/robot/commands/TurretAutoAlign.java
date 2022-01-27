// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;


public class TurretAutoAlign extends CommandBase {
  /** Creates a new TurretAutoAlign. */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  boolean tapeFound;
  SmartDashboard.putNumber("LimelightX", tx);
  SmartDashboard.putNumber("LimelightY", ty);
  SmartDashboard.putNumber("LimelightX", tv);

  if (tv != 1) {tapeFound = false;}
  else {tapeFound = true;}
  SmartDashboard.putBoolean("limelight vision", tapeFound);
  if (tapeFound = true){
    //Robot.driveSubsystem.teleop(0, 0);
    double heading_error = tx; //needs inverting?
    double steering_adjust = 0.0f;
    if (tx > 1.0)
    {
            steering_adjust = ShooterConstants.Kp * heading_error - ShooterConstants.min_command;
    }
    else if (tx < 1.0)
    {
            steering_adjust = ShooterConstants.Kp * heading_error + ShooterConstants.min_command;
    }
    Robot.turretSubsystem.turnTurret(steering_adjust);//toTest
  }}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
