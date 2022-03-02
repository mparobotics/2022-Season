// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoShootBall extends CommandBase {
  /** Creates a new AutoShootBall. */
  public AutoShootBall() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Timer.getMatchTime() < 7.0) {
    RobotContainer.shooterSub.shootBallAuto();
  
  }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSub.ShootBangBang(1);
    ElevatorSub.ElevateBall(ElevatorConstants.ELEVATOR_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
   if (Timer.getMatchTime() < 3.0){
    return true;}
    else {return false;}
  }
}
